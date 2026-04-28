#pragma once
#include "device_interface.hpp"
#include "ina228_driver.h"
#include "sensor_data.hpp"
#include "board_config.hpp"

namespace Devices 
{

class Ina228 : DeviceInterface
{
public:
    Ina228();
    DeviceStatus initializeDevice() override;
    DeviceStatus readPowerMeasurements(SensorData::PowerSample& sample);
private:
    PowerSensor::Ina228 device_;
};

} // namespace Devices