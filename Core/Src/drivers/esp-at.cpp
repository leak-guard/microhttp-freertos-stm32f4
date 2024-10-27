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

  void EspAtDriver::initTaskMain()
  {
    {
      auto lock = acquireLock();

      // Enable reception via DMA
      HAL_UART_Receive_DMA(m_usart, 
        reinterpret_cast<uint8_t*>(m_uartRxBuffer.data()), m_uartRxBuffer.size());

      sendCommandDirectAndWait("\r\n");

      sendCommandDirectAndWait("AT+RESTORE");
      xTaskNotifyWait(pdFALSE, ESP_READY, NULL, 1000);
      
      // ESP module is now ready

      sendCommandDirectAndWait("ATE0");
      sendCommandDirectAndWait("AT+SYSSTORE=0");
      sendCommandDirectAndWait("AT+CWMODE=2");
      auto resp = sendCommandDirectAndWait("AT+CWSAP=\"espat_test2\",\"passw0rd123\",5,3");
      HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
    }
    // Suspend self
    vTaskSuspend(NULL);
  }

  void EspAtDriver::uartRxTaskMain()
  {
    uint32_t dmaReadIdx = 0;
    StaticString<ESP_LINE_BUFFER_SIZE> lineBuffer;
    auto crlf = STR("\r\n");
    int ipdRemainingBytes = 0;

    volatile char* dmaBuffer = m_uartRxBuffer.data();

    while (true) {
      while (true) {
        uint32_t dmaWriteIdx = m_uartRxBuffer.size() - __HAL_DMA_GET_COUNTER(m_usart->hdmarx);
        if (dmaReadIdx == dmaWriteIdx) {
          // No more bytes to read
          break;
        }

        lineBuffer += dmaBuffer[dmaReadIdx++];

        if (lineBuffer.EndsWith(crlf)) {
          lineBuffer.Truncate(lineBuffer.GetSize() - 2);
          parseEspResponse(lineBuffer);
          lineBuffer.Clear();
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
      return finishRequest(EspResponse::SET_OK);
    }

    if (buffer == STR("ready")) {
      xTaskNotify(m_initTaskHandle, 1, eSetBits);
    }
  }

  void EspAtDriver::finishRequest(EspResponse response)
  {
    if (!m_requestInitiator) {
      return;
    }

    auto requestInitiator = m_requestInitiator;
    m_currentResponse = response;
    m_requestInitiator = nullptr;
    xTaskNotify(requestInitiator, ESP_RX_DONE, eSetBits);
  }

  auto EspAtDriver::sendCommandDirectAndWait(const char *data) -> EspResponse
  {
    static constexpr auto MAX_RESPONSE_WAIT_TIME = 1000; // Wait for 1s

    m_requestInitiator = xTaskGetCurrentTaskHandle();
    m_txLineBuffer = data;
    m_txLineBuffer += STR("\r\n");

    while (HAL_DMA_GetState(m_usart->hdmatx) != HAL_DMA_STATE_READY || m_usart->gState != HAL_UART_STATE_READY) {
      vTaskDelay(2);
    }

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

};
