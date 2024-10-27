#pragma once
#include <drivers/esp-at.hpp>

#include <optional>

namespace lg {

class Device {
public:
    static Device& get();

    void initializeDrivers();
    void setError();

    EspAtDriver& getEspAtDriver() { return m_espDriver; }

private:
    Device();

    static std::optional<Device> m_instance;
    
    EspAtDriver m_espDriver;
};

};
