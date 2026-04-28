#include "ina228.hpp"
#include "board_config.hpp"

namespace Devices
{

Ina228::Ina228()
{
    this->device_ = PowerSensor::Ina228(0x40, BoardConfig::PowerSensorHandle);
}

DeviceStatus Ina228::initializeDevice()
{
    return this->device_.begin() == false ? DeviceStatus::FAILED : DeviceStatus::OKAY;
}

DeviceStatus Ina228::readPowerMeasurements(SensorData::PowerSample& sample)
{
    sample.bus_voltage_V = device_.getBusVolt();
    sample.current_A = device_.getCurrent();
}

} // namespace Devices