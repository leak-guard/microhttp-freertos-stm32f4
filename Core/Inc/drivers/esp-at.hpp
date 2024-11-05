#pragma once
#include <leakguard/staticstring.hpp>

#include <array>
#include <cstdint>
#include <functional>

#include <FreeRTOS.h>
#include <semphr.h>
#include <stm32f4xx_hal.h>
#include <task.h>

#define ESP_AT_MAX_IPD_BYTES 2920
#define ESP_LINE_BUFFER_SIZE 512
#define ESP_RESPONSE_BUFFER_SIZE 2048
#define ESP_RX_DONE (1 << 0)
#define ESP_READY (1 << 1)

namespace lg
{

  class EspAtDriver
  {
  public:
    static inline constexpr auto MAX_CONNECTIONS = 5;
    static inline constexpr auto MAX_DATA_CHUNK_SIZE = 256;
    static inline constexpr auto MAX_INACTIVITY_TIME_MS = 5000;
    static inline constexpr auto DEFAULT_TIMEOUT = 3000;

    static void initTaskEntryPoint(void *params);
    static void uartRxTaskEntryPoint(void *params);
    static void connectionCloserEntryPoint(void *params);

    enum class EspWifiStatus 
    {
      DISCONNECTED,
      CONNECTING,
      CONNECTED,
      DHCP_GOT_IP,
    };

    enum class EspResponse
    {
      OK,
      ERROR,
      SEND_OK,
      SEND_FAIL,
      SET_OK,
      DRIVER_ERROR,
      ESP_TIMEOUT,
    };

    enum class EspWifiMode
    {
      OFF,
      STATION,
      AP,
      STATION_AND_AP,
    };

    EspAtDriver(UART_HandleTypeDef *iface)
      : m_usart(iface), m_ready(false), m_currentResponse(EspResponse::ESP_TIMEOUT), m_requestInitiator(nullptr), m_waitingForPrompt(false), m_ipdLinkId(0), m_ipdRemainingBytes(0), m_connectionOpen({false}), m_connectionLastActivity({0})
    {
    }

    void initialize();

    bool isReady() const { return m_ready; }
    EspWifiStatus getWifiStatus() const { return m_wifiStatus; }

    EspResponse startTcpServer(std::uint16_t portNumber);
    EspResponse stopTcpServer();
    EspResponse closeConnection(int linkId);
    EspResponse closeAllConnections();
    EspResponse sendData(int linkId, const char *data, std::size_t size);
    EspResponse setWifiMode(EspWifiMode mode);
    EspResponse joinAccessPoint(const char* ssid, const char* password);
    EspResponse listAccessPoints();

    std::function<void(int)> onConnected;
    std::function<void(int)> onClosed;
    std::function<void(int, const char *, std::size_t)> onData;

  private:
    class Lock
    {
    public:
      Lock(EspAtDriver *owner) : m_owner(owner) { xSemaphoreTake(owner->m_mutex, portMAX_DELAY); }
      ~Lock() { xSemaphoreGive(m_owner->m_mutex); }

    private:
      EspAtDriver *m_owner;
    };

    Lock acquireLock() { return Lock(this); }
    void initTaskMain();
    void uartRxTaskMain();
    void connectionCloserMain();

    void clearResponsePrefix();
    void setResponsePrefix(const char *data);
    void appendAtString(const char *data);

    void parseEspResponse(const StaticString<ESP_LINE_BUFFER_SIZE> &buffer);
    bool parseEspNotification(const StaticString<ESP_LINE_BUFFER_SIZE> &buffer);
    void finishRequest(EspResponse response);
    bool gotPrompt();
    void gotConnect(int linkId);
    void gotClosed(int linkId);
    void gotData(int linkId, const char *data, std::size_t size);
    void closeIdleConnections();

    void parseInputData(const StaticString<ESP_LINE_BUFFER_SIZE> &buffer);

    EspResponse sendCommandDirectAndWait(const char *data,
                                         std::uint32_t timeout = DEFAULT_TIMEOUT);
    EspResponse sendCommandBufferAndWait(std::uint32_t timeout = DEFAULT_TIMEOUT);
    void waitForDmaReady();

    volatile bool m_ready;

    UART_HandleTypeDef *m_usart;

    TaskHandle_t m_initTaskHandle;
    TaskHandle_t m_uartRxTaskHandle;
    TaskHandle_t m_connectionCloserTaskHandle;
    StaticTask_t m_initTaskTcb;
    StaticTask_t m_uartRxTaskTcb;
    StaticTask_t m_connectionCloserTaskTcb;
    std::array<configSTACK_DEPTH_TYPE, 64> m_initTaskStack;
    std::array<configSTACK_DEPTH_TYPE, 256> m_uartRxTaskStack;
    std::array<configSTACK_DEPTH_TYPE, 64> m_connectionCloserTaskStack;

    SemaphoreHandle_t m_mutex;
    StaticSemaphore_t m_mutexBuffer;

    StaticQueue_t m_connectionsToCloseQ;
    QueueHandle_t m_connectionsToCloseHandle;
    std::array<int, 16> m_connectionsToCloseBuffer;

    StaticString<ESP_LINE_BUFFER_SIZE> m_txLineBuffer;
    StaticString<32> m_responsePrefix;
    StaticString<ESP_RESPONSE_BUFFER_SIZE> m_responseBuffer;

    std::array<char, 16384> m_uartRxBuffer;
    std::array<volatile bool, MAX_CONNECTIONS> m_connectionOpen;
    std::array<volatile TickType_t, MAX_CONNECTIONS> m_connectionLastActivity;

    volatile EspResponse m_currentResponse;
    volatile TaskHandle_t m_requestInitiator;
    volatile bool m_waitingForPrompt;
    volatile EspWifiStatus m_wifiStatus;

    int m_ipdLinkId;
    uint32_t m_ipdRemainingBytes;
  };

};
