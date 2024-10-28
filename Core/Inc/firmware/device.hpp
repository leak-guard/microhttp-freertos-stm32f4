#pragma once
#include <drivers/esp-at.hpp>
#include <firmware/server.hpp>

#include <optional>

namespace lg
{

  class Device
  {
  public:
    static Device &get();

    Device();

    void initializeDrivers();
    void setError();

    EspAtDriver &getEspAtDriver() { return m_espDriver; }
    Server &getHttpServer() { return m_server; }

  private:
    static std::optional<Device> m_instance;

    EspAtDriver m_espDriver;
    Server m_server;
  };

};
