#include <firmware/device.hpp>

#include <usart.h>

namespace lg
{

  std::optional<Device> Device::m_instance;

  Device::Device()
      : m_espDriver(&huart1)
  {
  }

  Device &Device::get()
  {
    if (!m_instance.has_value())
    {
      m_instance.emplace(Device());
    }

    return m_instance.value();
  }

  void Device::initializeDrivers()
  {
    m_espDriver.initialize();
  }

  void Device::setError()
  {
    HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
  }

};
