#include <drivers/esp-at.hpp>

#include <firmware/device.hpp>

#include <array>
#include <cstring>

#include <gpio.h>

namespace lg
{

  void EspAtDriver::initTaskEntryPoint(void *params)
  {
    EspAtDriver *instance = reinterpret_cast<EspAtDriver *>(params);
    instance->initTaskMain();
  }

  void EspAtDriver::uartRxTaskEntryPoint(void *params)
  {
    EspAtDriver *instance = reinterpret_cast<EspAtDriver *>(params);
    instance->uartRxTaskMain();
  }

  void EspAtDriver::connectionCloserEntryPoint(void *params)
  {
    EspAtDriver *instance = reinterpret_cast<EspAtDriver *>(params);
    instance->connectionCloserMain();
  }

  void EspAtDriver::initialize()
  {
    m_requestInitiator = nullptr;

    m_initTaskHandle = xTaskCreateStatic(
      &EspAtDriver::initTaskEntryPoint /* Task function */,
      "ESP Init" /* Task name */,
      m_initTaskStack.size() /* Stack size */,
      this /* parameters */,
      configMAX_PRIORITIES - 1 /* Prority */,
      m_initTaskStack.data() /* Task stack address */,
      &m_initTaskTcb /* Task control block */
    );

    m_uartRxTaskHandle = xTaskCreateStatic(
      &EspAtDriver::uartRxTaskEntryPoint /* Task function */,
      "ESP Uart RX" /* Task name */,
      m_uartRxTaskStack.size() /* Stack size */,
      this /* parameters */,
      4 /* Prority */,
      m_uartRxTaskStack.data() /* Task stack address */,
      &m_uartRxTaskTcb /* Task control block */
    );

    m_connectionCloserTaskHandle = xTaskCreateStatic(
      &EspAtDriver::connectionCloserEntryPoint /* Task function */,
      "ESP Conn Closer" /* Task name */,
      m_connectionCloserTaskStack.size() /* Stack size */,
      this /* parameters */,
      2 /* Prority */,
      m_connectionCloserTaskStack.data() /* Task stack address */,
      &m_connectionCloserTaskTcb /* Task control block */
    );

    m_mutex = xSemaphoreCreateMutexStatic(&m_mutexBuffer);

    m_connectionsToCloseHandle = xQueueCreateStatic(
      m_connectionsToCloseBuffer.size(),
      sizeof(m_connectionsToCloseBuffer[0]),
      reinterpret_cast<std::uint8_t*>(m_connectionsToCloseBuffer.data()),
      &m_connectionsToCloseQ
    );
  }

  auto EspAtDriver::startTcpServer(std::uint16_t portNumber) -> EspResponse
  {
    auto lock = acquireLock();
    clearResponsePrefix();
    m_txLineBuffer = "AT+CIPSERVER=1,";
    m_txLineBuffer += StaticString<5>::Of(portNumber);
    m_txLineBuffer += "\r\n";
    return sendCommandBufferAndWait();
  }

  auto EspAtDriver::stopTcpServer() -> EspResponse
  {
    auto lock = acquireLock();
    clearResponsePrefix();
    return sendCommandDirectAndWait("AT+CIPSERVER=0,1");
  }

  auto EspAtDriver::closeConnection(int linkId) -> EspResponse
  {
    m_connectionOpen.at(linkId) = false;

    auto lock = acquireLock();
    clearResponsePrefix();
    m_txLineBuffer = "AT+CIPCLOSE=";
    m_txLineBuffer += StaticString<1>::Of(linkId);
    m_txLineBuffer += "\r\n";
    return sendCommandBufferAndWait();
  }

  auto EspAtDriver::closeAllConnections() -> EspResponse
  {
    static const auto falsch = false;
    m_connectionOpen.fill(falsch);

    auto lock = acquireLock();
    clearResponsePrefix();
    return sendCommandDirectAndWait("AT+CIPCLOSE=5");
  }

  auto EspAtDriver::sendData(
    int linkId, const char* data, std::size_t size) -> EspResponse
  {
    static constexpr auto MAX_PROMPT_WAIT_TIME = 1000; // Wait for 1s
    auto lock = acquireLock();

    m_connectionLastActivity.at(linkId) = xTaskGetTickCount();
    clearResponsePrefix();

    //xTaskNotifyWait(0, ESP_DATA_PROMPT, nullptr, 0);
    m_waitingForPrompt = true;
    m_txLineBuffer = "AT+CIPSEND=";
    m_txLineBuffer += StaticString<1>::Of(linkId);
    m_txLineBuffer += ',';
    m_txLineBuffer += StaticString<5>::Of(size);
    m_txLineBuffer += "\r\n";
    auto response = sendCommandBufferAndWait();
    if (response != EspResponse::OK) {
      m_waitingForPrompt = false;
      return response;
    }

    while (m_waitingForPrompt) {
      vTaskDelay(1);
    }

    // Now transmit data to ESP module
    m_requestInitiator = xTaskGetCurrentTaskHandle();

    waitForDmaReady();

    auto result = HAL_UART_Transmit_DMA(m_usart, 
      reinterpret_cast<const std::uint8_t*>(data), size);

    if (result != HAL_OK) {
      return EspResponse::DRIVER_ERROR;
    }

    bool gotRx = xTaskNotifyWait(0, ESP_RX_DONE, nullptr, MAX_PROMPT_WAIT_TIME);
    if (!gotRx) {
      return EspResponse::ESP_TIMEOUT;
    }

    return m_currentResponse;
  }

  auto EspAtDriver::setWifiMode(EspWifiMode mode) -> EspResponse
  {
    auto lock = acquireLock();
    clearResponsePrefix();
    m_txLineBuffer = "AT+CWMODE=";
    m_txLineBuffer += StaticString<1>::Of(static_cast<int>(mode));
    m_txLineBuffer += "\r\n";
    return sendCommandBufferAndWait();
  }

  auto EspAtDriver::joinAccessPoint(const char* ssid, const char* password) -> EspResponse
  {
    static constexpr auto COMMAND_TIMEOUT = 30000;
    auto lock = acquireLock();
    clearResponsePrefix();

    m_txLineBuffer = "AT+CWJAP=";
    appendAtString(ssid);
    m_txLineBuffer += ',';
    appendAtString(password);

    m_wifiStatus = EspWifiStatus::CONNECTING;

    auto result = sendCommandBufferAndWait(COMMAND_TIMEOUT);

    if (result != EspResponse::OK) {
      m_wifiStatus = EspWifiStatus::DISCONNECTED;
    }

    return result;
  }

  auto EspAtDriver::listAccessPoints(StaticVector<AccessPoint, ESP_AP_LIST_SIZE>& out) -> EspResponse
  {
    static constexpr auto COMMAND_TIMEOUT = 10000;

    auto lock = acquireLock();
    setResponsePrefix("+CWLAP:");

    auto response = sendCommandDirectAndWait("AT+CWLAP", COMMAND_TIMEOUT);

    if (response == EspResponse::OK) {
      out.Clear();
      std::size_t position = 0;

      while (out.GetSize() < out.GetCapacity()) {
        position += 8; // Length of "+CWLAP:("

        if (position >= m_responseBuffer.GetSize()) {
          break;
        }

        AccessPoint pnt;
        pnt.encryption = static_cast<Encryption>(readAtInteger(m_responseBuffer, position));
        readAtString(m_responseBuffer, position, pnt.ssid);
        pnt.rssi = readAtInteger(m_responseBuffer, position);
        readAtString(m_responseBuffer, position, pnt.mac);
        pnt.channel = readAtInteger(m_responseBuffer, position);
        pnt.freqOffset = readAtInteger(m_responseBuffer, position);
        pnt.freqCalVal = readAtInteger(m_responseBuffer, position);
        pnt.pairwiseCipher = static_cast<CipherType>(readAtInteger(m_responseBuffer, position));
        pnt.groupCipher = static_cast<CipherType>(readAtInteger(m_responseBuffer, position));
        pnt.bgn = readAtInteger(m_responseBuffer, position);
        pnt.wpsEnabled = readAtInteger(m_responseBuffer, position) != 0;

        out.Append(pnt);
      }
    }

    return response;
  }

  auto EspAtDriver::quitAccessPoint() -> EspResponse
  {
    auto lock = acquireLock();
    clearResponsePrefix();
    return sendCommandDirectAndWait("AT+CWQAP");
  }

  auto EspAtDriver::setupSoftAp(const char* ssid, 
      const char* password, int channel, Encryption encryption) -> EspResponse
  {
    auto lock = acquireLock();
    clearResponsePrefix();
    m_txLineBuffer = "AT+CWSAP=";
    appendAtString(ssid);
    m_txLineBuffer += ',';
    appendAtString(password);
    m_txLineBuffer += ',';
    m_txLineBuffer += StaticString<1>::Of(channel);
    m_txLineBuffer += ',';
    m_txLineBuffer += StaticString<1>::Of(static_cast<int>(encryption));
    m_txLineBuffer += "\r\n";
    return sendCommandBufferAndWait();
  }

  auto EspAtDriver::queryStationIp(StaticString<ESP_IP_STRING_SIZE>& out) -> EspResponse
  {
    auto lock = acquireLock();
    setResponsePrefix("+CIPSTA:ip:");

    auto response = sendCommandDirectAndWait("AT+CIPSTA?");

    if (response == EspResponse::OK) {
      m_responseBuffer.Skip(11); // Length of "+CIPSTA:ip:"
      m_responseBuffer.Truncate(m_responseBuffer.GetSize() - 2); //Cut off \r\n
      out = m_responseBuffer;
    }

    return response;
  }

  auto EspAtDriver::disableMdns() -> EspResponse
  {
    auto lock = acquireLock();
    clearResponsePrefix();
    return sendCommandDirectAndWait("AT+MDNS=0");
  }

  auto EspAtDriver::enableMdns(const char* hostname, const char* service, 
    std::uint16_t port, const char* instance, const char* proto, 
    const StaticVector<std::pair<const char*, const char*>, 8>& txtRecords) -> EspResponse
  {
    auto lock = acquireLock();
    clearResponsePrefix();
    m_txLineBuffer = "AT+MDNS=1,";
    appendAtString(hostname);
    m_txLineBuffer += ',';
    appendAtString(service);
    m_txLineBuffer += ',';
    m_txLineBuffer += StaticString<5>::Of(port);
    m_txLineBuffer += ',';
    appendAtString(instance);
    m_txLineBuffer += ',';
    appendAtString(proto);
    m_txLineBuffer += ',';
    m_txLineBuffer += StaticString<1>::Of(txtRecords.GetSize());

    for (const auto& record : txtRecords) {
      m_txLineBuffer += ',';
      appendAtString(record.first);
      m_txLineBuffer += ',';
      appendAtString(record.second);
    }

    m_txLineBuffer += "\r\n";
    return sendCommandBufferAndWait();
  }

  auto EspAtDriver::setHostname(const char* hostname) -> EspResponse
  {
    auto lock = acquireLock();
    clearResponsePrefix();

    m_txLineBuffer = "AT+CWHOSTNAME=";
    appendAtString(hostname);
    m_txLineBuffer += "\r\n";

    return sendCommandBufferAndWait();
  }

  void EspAtDriver::initTaskMain()
  {
    {
      auto lock = acquireLock();

      // Enable reception via DMA
      HAL_UART_Receive_DMA(m_usart, 
        reinterpret_cast<uint8_t*>(m_uartRxBuffer.data()), m_uartRxBuffer.size());

      vTaskDelay(1000);
      sendCommandDirectAndWait("");
      
      // ESP module is now ready

      sendCommandDirectAndWait("ATE0");
      sendCommandDirectAndWait("AT+SYSSTORE=0");
      //sendCommandDirectAndWait("AT+CWMODE=3");
      sendCommandDirectAndWait("AT+CWMODE=2");
      sendCommandDirectAndWait("AT+CIPMODE=0");
      sendCommandDirectAndWait("AT+CIPMUX=1");
      sendCommandDirectAndWait("AT+CIPDINFO=0");
      sendCommandDirectAndWait("AT+CIPV6=0");
      sendCommandDirectAndWait("AT+CIPRECVTYPE=5,0");
      auto resp = sendCommandDirectAndWait("AT+CWSAP=\"espat_test2\",\"passw0rd123\",5,3");
      //auto resp = EspResponse::OK;

      if (resp != EspResponse::OK) {
        Device::get().setError();
      } else {
        m_ready = true;
      }
    }
    setHostname("leakguardtest");

    StaticVector<std::pair<const char*, const char*>, 8> records;
    records.Append(std::make_pair("meta", "some data"));
    enableMdns("espik", "leakguard", 80, "espik2", "_tcp", records);

    // Suspend self
    vTaskSuspend(NULL);
  }

  void EspAtDriver::uartRxTaskMain()
  {
    uint32_t dmaReadIdx = 0;
    StaticString<ESP_LINE_BUFFER_SIZE> lineBuffer;
    auto crlf = STR("\r\n");
    auto colon = STR(":");

    volatile char* dmaBuffer = m_uartRxBuffer.data();

    while (true) {
      while (true) {
        closeIdleConnections();
        
        uint32_t dmaWriteIdx = m_uartRxBuffer.size() - __HAL_DMA_GET_COUNTER(m_usart->hdmarx);
        if (dmaReadIdx == dmaWriteIdx) {
          // No more bytes to read
          break;
        }

        if (m_ipdRemainingBytes > 0) {
          uint32_t bytesInBuffer = 0;
          bool exit = true;

          if (dmaWriteIdx > dmaReadIdx) {
            bytesInBuffer = dmaWriteIdx - dmaReadIdx;
          } else {
            bytesInBuffer = m_uartRxBuffer.size() - dmaReadIdx;
            exit = false;
          }

          if (m_ipdRemainingBytes < bytesInBuffer) {
            bytesInBuffer = m_ipdRemainingBytes;
          }

          if (bytesInBuffer > MAX_DATA_CHUNK_SIZE) {
            bytesInBuffer = MAX_DATA_CHUNK_SIZE;
            exit = false;
          }

          gotData(m_ipdLinkId, m_uartRxBuffer.begin() + dmaReadIdx, bytesInBuffer);
          m_ipdRemainingBytes -= bytesInBuffer;
          dmaReadIdx += bytesInBuffer;

          if (dmaReadIdx >= m_uartRxBuffer.size()) {
            dmaReadIdx = 0;
          }

          if (exit) {
            break;
          }

          continue;
        }

        lineBuffer += dmaBuffer[dmaReadIdx++];
        if (dmaReadIdx >= m_uartRxBuffer.size()) {
          dmaReadIdx = 0;
        }

        if (lineBuffer.EndsWith(crlf)) {
          lineBuffer.Truncate(lineBuffer.GetSize() - 2);
          parseEspResponse(lineBuffer);
          lineBuffer.Clear();
        } else if (lineBuffer.EndsWith(colon)) {
          if (parseEspNotification(lineBuffer)) {
            lineBuffer.Clear();
          }
        } else if (lineBuffer.GetSize() == 1 && *lineBuffer.begin() == '>') {
          if (gotPrompt()) {
            lineBuffer.Clear();
          }
        }
      }

      vTaskDelay(5);
    }
  }

  void EspAtDriver::connectionCloserMain()
  {
    while (true) {
      int connectionId;
      if (xQueueReceive(m_connectionsToCloseHandle, &connectionId, portMAX_DELAY) == pdPASS) {
        closeConnection(connectionId);
      }
    }
  }

  void EspAtDriver::clearResponsePrefix()
  {
    taskENTER_CRITICAL();
    m_responsePrefix.Clear();
    taskEXIT_CRITICAL();
  }

  void EspAtDriver::setResponsePrefix(const char* data)
  {
    taskENTER_CRITICAL();
    m_responsePrefix = data;
    m_responseBuffer.Clear();
    taskEXIT_CRITICAL();
  }
  
  void EspAtDriver::appendAtString(const char* data)
  {
    m_txLineBuffer += '"';

    while (true) {
      char c = *data;

      if (!c) {
        break;
      }

      if (c == '\\' || c == ',' || c == '"') {
        // Add backslash for escaping characters
        m_txLineBuffer += '\\';
      }

      m_txLineBuffer += c;
      ++data;
    }

    m_txLineBuffer += '"';
  }
  
  int EspAtDriver::readAtInteger(
    const StaticString<ESP_RESPONSE_BUFFER_SIZE>& buffer, std::size_t& position)
  {
    std::size_t currentPos = position;
    bool negative = false;
    int out = 0;

    if (position < buffer.GetSize() && buffer[currentPos] == '-') {
      negative = true;
      ++currentPos;
    }

    while (true) {
      if (position >= buffer.GetSize()) {
        break;
      }

      char c = buffer[currentPos];
      if (c == ',' || c == '\r' || c == '\n') {
        while (c == ',' || c == '\r' || c == '\n') {
          ++currentPos;
          c = buffer[currentPos];
        }

        break;
      }

      if (std::isdigit(c)) {
        out *= 10;
        out += c - '0';
      }

      ++currentPos;
    }

    position = currentPos;
    return negative ? -out : out;
  }
    
  template <std::size_t outSize>
  void EspAtDriver::readAtString(
    const StaticString<ESP_RESPONSE_BUFFER_SIZE>& buffer, 
    std::size_t& position, StaticString<outSize>& out)
  {
    std::size_t currentPos = position;
    bool isEscaping = false;
    out.Clear();

    if (position >= buffer.GetSize() || buffer[position] != '"') {
      return;
    }

    ++currentPos;

    while (true) {
      if (position >= buffer.GetSize()) {
        break;
      }

      char c = buffer[currentPos];
      if (isEscaping) {
        out += c;
        isEscaping = false;
      } else if (c == '\\') {
        isEscaping = true;
      } else if (c == '"') {
        ++currentPos;
        break;
      } else {
        out += c;
      }

      ++currentPos;
    }

    while (currentPos < buffer.GetSize()) {
      char c = buffer[currentPos];

      if (!(c == ',' || c == '\r' || c == '\n')) {
        break;
      }

      ++currentPos;
    }

    position = currentPos;
  }

  void EspAtDriver::parseEspResponse(
    const StaticString<ESP_LINE_BUFFER_SIZE>& buffer)
  {
    if (buffer == STR("OK")) {
      return finishRequest(EspResponse::OK);
    }

    if (buffer == STR("ERROR")) {
      return finishRequest(EspResponse::ERROR);
    }

    if (buffer == STR("SEND OK")) {
      return finishRequest(EspResponse::SEND_OK);
    }

    if (buffer == STR("SEND FAIL")) {
      return finishRequest(EspResponse::SEND_FAIL);
    }

    if (buffer == STR("SET OK")) {
      return finishRequest(EspResponse::SET_OK);
    }

    if (buffer == STR("ready")) {
      xTaskNotify(m_initTaskHandle, 1, eSetBits);
      return;
    }

    if (buffer == STR("WIFI CONNECTED")) {
      m_wifiStatus = EspWifiStatus::CONNECTED;
      return;
    }

    if (buffer == STR("WIFI GOT IP")) {
      m_wifiStatus = EspWifiStatus::DHCP_GOT_IP;
      return;
    }

    if (buffer == STR("WIFI DISCONNECT")) {
      m_wifiStatus = EspWifiStatus::DISCONNECTED;
      return;
    }

    if (buffer.EndsWith(STR("CONNECT"))) {
      if (buffer[1] == ',') {
        int linkId = buffer[0] - '0';
        gotConnect(linkId);
        return;
      }
      return gotConnect(0);
    }

    if (buffer.EndsWith(STR("CLOSED"))) {
      if (buffer[1] == ',') {
        int linkId = buffer[0] - '0';
        gotClosed(linkId);
        return;
      }
      return gotClosed(0);
    }

    if (!m_responsePrefix.IsEmpty() && buffer.StartsWith(m_responsePrefix)) {
      m_responseBuffer += buffer;
      m_responseBuffer += "\r\n";
    }
  }

  bool EspAtDriver::parseEspNotification(
    const StaticString<ESP_LINE_BUFFER_SIZE>& buffer)
  {
    if (buffer.StartsWith(STR("+IPD,"))) {
      parseInputData(buffer);
      return true;
    }

    if (buffer.StartsWith(STR("+MQTTSUB:"))) {
      // TODO: parse MQTT events
      return true;
    }

    return false;
  }

  void EspAtDriver::finishRequest(EspResponse response)
  {
    auto requestInitiator = m_requestInitiator;
    if (!requestInitiator) {
      return;
    }

    m_currentResponse = response;

    if (!m_waitingForPrompt) {
      m_requestInitiator = nullptr;
    }

    xTaskNotify(requestInitiator, ESP_RX_DONE, eSetBits);
  }

  bool EspAtDriver::gotPrompt()
  {
    auto requestInitiator = m_requestInitiator;
    if (!requestInitiator) {
      return false;
    }

    if (m_waitingForPrompt) {
      m_requestInitiator = nullptr;
      m_waitingForPrompt = false;

      return true;
    }

    return false;
  }

  void EspAtDriver::gotConnect(int linkId)
  {
    if (linkId >= 0 && linkId < MAX_CONNECTIONS) {
      m_connectionOpen.at(linkId) = true;
      m_connectionLastActivity.at(linkId) = xTaskGetTickCount();
    }

    if (onConnected) {
      onConnected(linkId);
    }
  }

  void EspAtDriver::gotClosed(int linkId)
  {
    if (linkId >= 0 && linkId < MAX_CONNECTIONS) {
      m_connectionOpen.at(linkId) = false;
    }

    if (onClosed) {
      onClosed(linkId);
    }
  }

  void EspAtDriver::gotData(int linkId, const char* data, std::size_t size)
  {
    if (linkId >= 0 && linkId < MAX_CONNECTIONS) {
      m_connectionLastActivity.at(linkId) = xTaskGetTickCount();
    }

    if (onData) {
      onData(linkId, data, size);
    }
  }

  void EspAtDriver::closeIdleConnections()
  {
    TickType_t currentTick = xTaskGetTickCount();

    for (size_t i = 0; i < MAX_CONNECTIONS; ++i) {
      if (m_connectionOpen.at(i)) {
        if (currentTick - m_connectionLastActivity.at(i) > MAX_INACTIVITY_TIME_MS) {
          int connectionId = i;
          xQueueSend(m_connectionsToCloseHandle, &connectionId, 0);
          gotClosed(connectionId);
        }
      }
    }
  }

  void EspAtDriver::parseInputData(const StaticString<ESP_LINE_BUFFER_SIZE>& buffer)
  {
    StaticString<16> bufferCopy = buffer;
    const char* bufferData = bufferCopy.ToCStr();

    int linkId = atoi(bufferData + 5);
    int dataSize = atoi(bufferData + 7);

    if (linkId >= 0 && linkId < MAX_CONNECTIONS 
      && dataSize >= 0 && dataSize <= ESP_AT_MAX_IPD_BYTES) {

      m_ipdLinkId = linkId;
      m_ipdRemainingBytes = dataSize;
    }
  }

  auto EspAtDriver::sendCommandDirectAndWait(const char *data, std::uint32_t timeout) -> EspResponse
  {
    m_txLineBuffer = data;
    m_txLineBuffer += "\r\n";

    return sendCommandBufferAndWait(timeout);
  }

  auto EspAtDriver::sendCommandBufferAndWait(std::uint32_t timeout) -> EspResponse
  {
    m_requestInitiator = xTaskGetCurrentTaskHandle();

    waitForDmaReady();

    auto result = HAL_UART_Transmit_DMA(m_usart, 
      reinterpret_cast<const std::uint8_t*>(m_txLineBuffer.begin()), m_txLineBuffer.GetSize());

    if (result != HAL_OK) {
      return EspResponse::DRIVER_ERROR;
    }

    bool gotRx = xTaskNotifyWait(0, ESP_RX_DONE, nullptr, timeout);
    if (!gotRx) {
      return EspResponse::ESP_TIMEOUT;
    }

    return m_currentResponse;
  }

  void EspAtDriver::waitForDmaReady()
  {
    while (HAL_DMA_GetState(m_usart->hdmatx) != HAL_DMA_STATE_READY 
      || m_usart->gState != HAL_UART_STATE_READY) {

      vTaskDelay(2);
    }
  }

};
