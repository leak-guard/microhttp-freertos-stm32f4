#pragma once
#include <leakguard/microhttp.hpp>

#include <drivers/esp-at.hpp>

#include <array>

#include <FreeRTOS.h>
#include <message_buffer.h>
#include <task.h>

namespace lg
{
  struct SocketMessage {
    enum class MessageType {
      CONNECTED,
      DISCONNECTED,
      DATA
    };

    MessageType messageType;
    char data[];
  };

  class EspSocketImpl
  {
  public:
    static inline constexpr auto  MAX_CONNECTIONS = EspAtDriver::MAX_CONNECTIONS;
    static inline constexpr auto BUFFER_SIZE = 1024;

    static void workerEntryPoint(void* params);

    EspSocketImpl(HttpServerBase& server);

    void init();
    void bind(std::uint16_t port);
    void close(int connectionId);
    std::size_t send(int connectionId, const char *data, std::size_t numBytes);
    void finish(int connectionId);

  private:
    struct WorkerParams {
      EspSocketImpl* instance;
      int workerId;
    };
    
    void workerMain(int workerId);

    HttpServerBase& m_server;
    EspAtDriver* m_esp;

    std::array<StaticTask_t, MAX_CONNECTIONS> m_workerTaskTcb;
    std::array<TaskHandle_t, MAX_CONNECTIONS> m_workerTaskHandle;
    std::array<std::array<configSTACK_DEPTH_TYPE, 1024>, MAX_CONNECTIONS> m_workerTaskStack;
    std::array<std::array<std::uint8_t, BUFFER_SIZE>, MAX_CONNECTIONS> m_workerTaskBufferBlock;
    std::array<StaticMessageBuffer_t, MAX_CONNECTIONS> m_workerTaskMessageBuffer;
    std::array<MessageBufferHandle_t, MAX_CONNECTIONS> m_workerTaskMessageBufferHandle;
    std::array<WorkerParams, MAX_CONNECTIONS> m_workerParams;
  };

};
