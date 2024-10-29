#include <firmware/server.hpp>

#include <stm32f411xe.h>

namespace lg {

  void Server::initHttpEntryPoint(void* params)
  {
    Server *instance = reinterpret_cast<Server *>(params);
    instance->initHttpMain();
  }

  void Server::initialize()
  {
    m_httpRootTaskHandle = xTaskCreateStatic(
      &Server::initHttpEntryPoint /* Task function */,
      "HTTP Init Task" /* Task name */,
      m_httpRootTaskStack.size() /* Stack size */,
      this /* parameters */,
      configMAX_PRIORITIES - 1 /* Prority */,
      m_httpRootTaskStack.data() /* Task stack address */,
      &m_httpRootTaskTcb /* Task control block */
    );
  }

  void Server::initHttpMain()
  {
    m_server.get("/hello", [&](Request& req, Response& res) {
      uint32_t id[3] = { HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2() };

      res << "<h1>It works!</h1><p>The STM32 device id is: <b>";
      res << id[0] << '-' << id[1] << '-' << id[2] << "</b></p>";
    });

    m_server.get("/test/:1", [&](Request& req, Response& res) {
      res << "I've got a parameter: " << req.params[1];
    });

    m_server.start();
  }

};
