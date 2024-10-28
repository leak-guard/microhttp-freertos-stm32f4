#pragma once
#include <leakguard/staticstring.hpp>

#include <array>
#include <cstdint>

#include <FreeRTOS.h>
#include <semphr.h>
#include <stm32f4xx_hal.h>
#include <task.h>


#define ESP_AT_MAX_IPD_BYTES  2920
#define ESP_LINE_BUFFER_SIZE  512
#define ESP_RX_DONE           (1 << 0)
#define ESP_READY             (1 << 1)

namespace lg {

class EspAtDriver {
public:
    static void initTaskEntryPoint(void* params);
    static void uartRxTaskEntryPoint(void* params);

    enum class EspResponse {
        OK,
        ERROR,
        SEND_OK,
        SEND_FAIL,
        SET_OK,
        DRIVER_ERROR,
        ESP_TIMEOUT,
    };

    EspAtDriver(UART_HandleTypeDef* iface)
        : m_usart(iface)
        , m_ready(false)
    {
    }

    void initialize();

    bool isReady() const { return m_ready; }
    EspResponse startTcpServer(std::uint16_t portNumber);
    EspResponse stopTcpServer();

private:
    class Lock {
    public:
        Lock(EspAtDriver* owner) : m_owner(owner) { xSemaphoreTake(owner->m_mutex, portMAX_DELAY); }
        ~Lock() { xSemaphoreGive(m_owner->m_mutex); }

    private:
        EspAtDriver* m_owner;
    };

    Lock acquireLock() { return Lock(this); }
    void initTaskMain();
    void uartRxTaskMain();

    void parseEspResponse(const StaticString<ESP_LINE_BUFFER_SIZE>& buffer);
    void finishRequest(EspResponse response);

    EspResponse sendCommandDirectAndWait(const char* data);
    EspResponse sendCommandBufferAndWait();

    volatile bool m_ready;

    UART_HandleTypeDef* m_usart;
    TaskHandle_t m_initTaskHandle;
    TaskHandle_t m_uartRxTaskHandle;
    StaticTask_t m_initTaskTcb;
    StaticTask_t m_uartRxTaskTcb;
    std::array<configSTACK_DEPTH_TYPE, 64> m_initTaskStack;
    std::array<configSTACK_DEPTH_TYPE, 256> m_uartRxTaskStack;

    SemaphoreHandle_t m_mutex;
    StaticSemaphore_t m_mutexBuffer;

    StaticString<ESP_LINE_BUFFER_SIZE> m_txLineBuffer;
    std::array<char, 16384> m_uartRxBuffer;
    volatile EspResponse m_currentResponse;
    volatile TaskHandle_t m_requestInitiator;
};

};
