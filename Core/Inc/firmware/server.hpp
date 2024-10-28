#pragma once
#include "socket.hpp"

#include <leakguard/microhttp.hpp>

#include <FreeRTOS.h>

namespace lg {

  class Server {
  public:
    static void initHttpEntryPoint(void* params);

    using Server_t = HttpServer<EspSocketImpl, EspSocketImpl::MAX_CONNECTIONS>;
    using Request = Server_t::Request;
    using Response = Server_t::Response;

    Server() = default;

    void initialize();

  private:
    void initHttpMain();

    Server_t m_server;

    TaskHandle_t m_httpRootTaskHandle;
    StaticTask_t m_httpRootTaskTcb;
    std::array<configSTACK_DEPTH_TYPE, 512> m_httpRootTaskStack;
  };

};
