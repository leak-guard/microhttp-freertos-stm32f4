#include <firmware/socket.hpp>

#include <firmware/device.hpp>

#include <gpio.h>

namespace lg {

  void EspSocketImpl::workerEntryPoint(void* params)
  {
    WorkerParams* workerParams = reinterpret_cast<WorkerParams*>(params);
    workerParams->instance->workerMain(workerParams->workerId);
  }

  EspSocketImpl::EspSocketImpl(HttpServerBase& server)
    : m_server(server)
    , m_esp(nullptr)
  {
  }

  void EspSocketImpl::init()
  {
    for (std::size_t i = 0; i < MAX_CONNECTIONS; ++i) {
      StaticString<16> taskName = "HTTP Worker ";
      taskName += StaticString<8>::Of(i);

      m_workerParams.at(i) = { .instance = this, .workerId = static_cast<int>(i) };

      m_workerTaskHandle.at(i) = xTaskCreateStatic(
        &EspSocketImpl::workerEntryPoint /* Task function */,
        taskName.ToCStr() /* Task name */,
        m_workerTaskStack.at(i).size() /* Stack size */,
        &m_workerParams.at(i) /* parameters */,
        3 /* Prority */,
        m_workerTaskStack.at(i).data() /* Task stack address */,
        &m_workerTaskTcb.at(i) /* Task control block */
      );

      m_workerTaskMessageBufferHandle.at(i) = xMessageBufferCreateStatic(
        m_workerTaskBufferBlock.at(i).size(),
        m_workerTaskBufferBlock.at(i).data(),
        &m_workerTaskMessageBuffer.at(i)
      );
    }
  }

  void EspSocketImpl::bind(std::uint16_t port)
  {
    m_esp = &Device::get().getEspAtDriver();

    m_esp->onConnected = [&](int linkId) {
      if (linkId >= 0 && linkId < MAX_CONNECTIONS) {
        SocketMessage msg;
        msg.messageType = SocketMessage::MessageType::CONNECTED;
        xMessageBufferSend(m_workerTaskMessageBufferHandle.at(linkId), &msg, sizeof(SocketMessage), portMAX_DELAY);
      } else {
        Device::get().setError();
      }
    };

    m_esp->onClosed = [&](int linkId) {
      if (linkId >= 0 && linkId < MAX_CONNECTIONS) {
        SocketMessage msg;
        msg.messageType = SocketMessage::MessageType::DISCONNECTED;
        xMessageBufferSend(m_workerTaskMessageBufferHandle.at(linkId), &msg, sizeof(SocketMessage), portMAX_DELAY);
      } else {
        Device::get().setError();
      }
    };

    m_esp->onData = [&](int linkId, const char* data, std::size_t size) {
      if (linkId >= 0 && linkId < MAX_CONNECTIONS && size <= EspAtDriver::MAX_DATA_CHUNK_SIZE) {
        std::array<uint8_t, sizeof(SocketMessage) + EspAtDriver::MAX_DATA_CHUNK_SIZE> buffer;
        SocketMessage* msg = reinterpret_cast<SocketMessage*>(buffer.data());
        msg->messageType = SocketMessage::MessageType::DATA;
        std::memcpy(msg->data, data, size);

        std::size_t msgSize = sizeof(SocketMessage) + size;
        xMessageBufferSend(m_workerTaskMessageBufferHandle.at(linkId), buffer.data(), msgSize, portMAX_DELAY);
      } else {
        Device::get().setError();
      }
    };


    // Wait until ESP module is ready
    while (!m_esp->isReady()) {
      vTaskDelay(50);
    }
    
    auto response = m_esp->startTcpServer(port);
    if (response != EspAtDriver::EspResponse::OK) {
      Device::get().setError();
    }

    vTaskSuspend(nullptr);
  }

  void EspSocketImpl::close(int connectionId)
  {
    m_esp->closeConnection(connectionId);
  }

  std::size_t EspSocketImpl::send(
    int connectionId, const char *data, std::size_t numBytes)
  {
    auto response = m_esp->sendData(connectionId, data, numBytes);
    if (response != EspAtDriver::EspResponse::SEND_OK) {
      HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
      vTaskDelay(5);
      return 0;
    }

    return numBytes;
  }

  void EspSocketImpl::finish(int connectionId)
  {
    // Stub
  }

  void EspSocketImpl::workerMain(int workerId)
  {
    std::array<char, BUFFER_SIZE - sizeof(std::uint32_t)> rxBuffer;
    SocketMessage* msg = reinterpret_cast<SocketMessage*>(rxBuffer.data());

    while (true) {
      auto rxSize = xMessageBufferReceive(
        m_workerTaskMessageBufferHandle.at(workerId),
        rxBuffer.data(),
        rxBuffer.size(),
        portMAX_DELAY
      );

      std::size_t dataSize = rxSize - sizeof(SocketMessage);

      switch (msg->messageType) {
      case SocketMessage::MessageType::CONNECTED:
        m_server.clientConnected(workerId);
        break;
      case SocketMessage::MessageType::DISCONNECTED:
        m_server.clientDisconnected(workerId);
        break;
      case SocketMessage::MessageType::DATA:
        m_server.recvBytes(workerId, msg->data, dataSize);
        break;
      }
    }
  }

};
