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

    m_mutex = xSemaphoreCreateMutexStatic(&m_mutexBuffer);
  }

  auto EspAtDriver::startTcpServer(std::uint16_t portNumber) -> EspResponse
  {
    auto lock = acquireLock();
    m_txLineBuffer = "AT+CIPSERVER=1,";
    m_txLineBuffer += StaticString<5>::Of(portNumber);
    m_txLineBuffer += "\r\n";
    return sendCommandBufferAndWait();
  }

  auto EspAtDriver::stopTcpServer() -> EspResponse
  {
    auto lock = acquireLock();
    return sendCommandDirectAndWait("AT+CIPSERVER=0,1");
  }

  auto EspAtDriver::closeConnection(int linkId) -> EspResponse
  {
    auto lock = acquireLock();
    m_txLineBuffer = "AT+CIPCLOSE=";
    m_txLineBuffer += StaticString<1>::Of(linkId);
    m_txLineBuffer += "\r\n";
    return sendCommandBufferAndWait();
  }

  auto EspAtDriver::closeAllConnections() -> EspResponse
  {
    auto lock = acquireLock();
    return sendCommandDirectAndWait("AT+CIPCLOSE=5");
  }

  auto EspAtDriver::sendData(
    int linkId, const char* data, std::size_t size) -> EspResponse
  {
    static constexpr auto MAX_PROMPT_WAIT_TIME = 1000; // Wait for 1s
    auto lock = acquireLock();

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

    /*bool gotPrompt = xTaskNotifyWait(0, ESP_DATA_PROMPT, nullptr, MAX_PROMPT_WAIT_TIME);
    if (!gotPrompt) {
      m_waitingForPrompt = false;
      return EspResponse::ESP_TIMEOUT;
    }*/

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

      sendCommandDirectAndWait("ATE1");
      sendCommandDirectAndWait("AT+SYSSTORE=0");
      sendCommandDirectAndWait("AT+CWMODE=2");
      sendCommandDirectAndWait("AT+CIPMODE=0");
      sendCommandDirectAndWait("AT+CIPMUX=1");
      sendCommandDirectAndWait("AT+CIPDINFO=0");
      sendCommandDirectAndWait("AT+CIPV6=0");
      sendCommandDirectAndWait("AT+CIPRECVTYPE=5,0");
      auto resp = sendCommandDirectAndWait("AT+CWSAP=\"espat_test2\",\"passw0rd123\",5,3");

      if (resp != EspResponse::OK) {
        Device::get().setError();
      } else {
        m_ready = true;
      }
    }
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

    if (buffer.EndsWith(STR("CONNECT"))) {
      if (buffer.begin()[1] == ',') {
        int linkId = buffer.begin()[0] - '0';
        gotConnect(linkId);
        return;
      }
      return gotConnect(0);
    }

    if (buffer.EndsWith(STR("CLOSED"))) {
      if (buffer.begin()[1] == ',') {
        int linkId = buffer.begin()[0] - '0';
        gotClosed(linkId);
        return;
      }
      return gotClosed(0);
    }
  }

  bool EspAtDriver::parseEspNotification(
    const StaticString<ESP_LINE_BUFFER_SIZE>& buffer)
  {
    if (buffer.StartsWith(STR("+IPD,"))) {
      parseInputData(buffer);
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
    if (onConnected) {
      onConnected(linkId);
    }
  }

  void EspAtDriver::gotClosed(int linkId)
  {
    if (onClosed) {
      onClosed(linkId);
    }
  }

  void EspAtDriver::gotData(int linkId, const char* data, std::size_t size)
  {
    if (onData) {
      onData(linkId, data, size);
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

  auto EspAtDriver::sendCommandDirectAndWait(const char *data) -> EspResponse
  {
    m_txLineBuffer = data;
    m_txLineBuffer += "\r\n";

    return sendCommandBufferAndWait();
  }

  auto EspAtDriver::sendCommandBufferAndWait() -> EspResponse
  {
    static constexpr auto MAX_RESPONSE_WAIT_TIME = 1000; // Wait for 1s
    
    m_requestInitiator = xTaskGetCurrentTaskHandle();

    waitForDmaReady();

    auto result = HAL_UART_Transmit_DMA(m_usart, 
      reinterpret_cast<const std::uint8_t*>(m_txLineBuffer.begin()), m_txLineBuffer.GetSize());

    if (result != HAL_OK) {
      return EspResponse::DRIVER_ERROR;
    }

    bool gotRx = xTaskNotifyWait(0, ESP_RX_DONE, nullptr, MAX_RESPONSE_WAIT_TIME);
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
