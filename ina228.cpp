//    FILE: INA228.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.4.1
//    DATE: 2024-05-09
// PURPOSE: Arduino library for the INA228, I2C, 20 bit, voltage, current and power sensor.
//     URL: https://github.com/RobTillaart/INA228
//          https://www.adafruit.com/product/5832           ( 10 A version)
//          https://www.mateksys.com/?portfolio=i2c-ina-bm  (200 A version))

//
//  Read the datasheet for the details


#include "ina228.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include <cmath>

//      REGISTERS                   ADDRESS    BITS  RW
#define INA228_CONFIG               0x00    //  16   RW
#define INA228_ADC_CONFIG           0x01    //  16   RW
#define INA228_SHUNT_CAL            0x02    //  16   RW
#define INA228_SHUNT_TEMP_CO        0x03    //  16   RW
#define INA228_SHUNT_VOLTAGE        0x04    //  24   R-
#define INA228_BUS_VOLTAGE          0x05    //  24   R-
#define INA228_TEMPERATURE          0x06    //  16   R-
#define INA228_CURRENT              0x07    //  24   R-
#define INA228_POWER                0x08    //  24   R-
#define INA228_ENERGY               0x09    //  40   R-
#define INA228_CHARGE               0x0A    //  40   R-
#define INA228_DIAG_ALERT           0x0B    //  16   RW
#define INA228_SOVL                 0x0C    //  16   RW
#define INA228_SUVL                 0x0D    //  16   RW
#define INA228_BOVL                 0x0E    //  16   RW
#define INA228_BUVL                 0x0F    //  16   RW
#define INA228_TEMP_LIMIT           0x10    //  16   RW
#define INA228_POWER_LIMIT          0x11    //  16   RW
#define INA228_MANUFACTURER         0x3E    //  16   R-
#define INA228_DEVICE_ID            0x3F    //  16   R-


//  CONFIG MASKS (register 0)
#define INA228_CFG_RST              0x8000
#define INA228_CFG_RSTACC           0x4000
#define INA228_CFG_CONVDLY          0x3FC0
#define INA228_CFG_TEMPCOMP         0x0020
#define INA228_CFG_ADCRANGE         0x0010
#define INA228_CFG_RESERVED         0x000F  //  all unused bits


//  ADC MASKS (register 1)
#define INA228_ADC_MODE             0xF000
#define INA228_ADC_VBUSCT           0x0E00
#define INA228_ADC_VSHCT            0x01C0
#define INA228_ADC_VTCT             0x0038
#define INA228_ADC_AVG              0x0007


////////////////////////////////////////////////////////
//
//  CONSTRUCTOR
//
Ina228::Ina228(const uint8_t address, I2C_HandleTypeDef *i2c_handle)
{
  _address     = address;
  _i2c_handle  = i2c_handle;
  //  no calibrated values by default.
  _shunt       = 0.015;
  _maxCurrent  = 10.0;
  _current_LSB = _maxCurrent * std::pow(2, -19);
  _error       = 0;
}


bool Ina228::begin()
{
  if (! isConnected()) return false;

  getADCRange();
  return true;
}


bool Ina228::isConnected()
{
  return HAL_I2C_IsDeviceReady(
      _i2c_handle, 
      _address << 1,
      1, 
      100
    ) == HAL_OK; 
}


uint8_t Ina228::getAddress()
{
  return _address;
}


////////////////////////////////////////////////////////
//
//  CORE FUNCTIONS
//
//  PAGE 25
float Ina228::getBusVoltage()
{
  //  always positive, remove reserved bits.
  int32_t value = _readRegister(INA228_BUS_VOLTAGE, 3) >> 4;
  float bus_LSB = 195.3125e-6;  //  195.3125 uV
  float voltage = value * bus_LSB;
  return voltage;
}

//  PAGE 25
float Ina228::getShuntVoltage()
{
  //  shunt_LSB depends on ADCRANGE in INA228_CONFIG register.
  float shunt_LSB = 312.5e-9;  //  312.5 nV
  if (_ADCRange == true)
  {
    shunt_LSB = 78.125e-9;     //  78.125 nV
  }

  //  remove reserved bits.
  int32_t value = _readRegister(INA228_SHUNT_VOLTAGE, 3) >> 4;
  //  handle negative values (20 bit)
  if (value & 0x00080000)
  {
    value |= 0xFFF00000;
  }
  float voltage = value * shunt_LSB;
  return voltage;
}

int32_t Ina228::getShuntVoltageRAW()
{
  //  remove reserved bits.
  uint32_t value = _readRegister(INA228_SHUNT_VOLTAGE, 3) >> 4;
  //  handle negative values (20 bit)
  if (value & 0x00080000)
  {
    value |= 0xFFF00000;
  }
  return (int32_t)value;
}

//  PAGE 25 + 8.1.2
float Ina228::getCurrent()
{
  //  remove reserved bits.
  int32_t value = _readRegister(INA228_CURRENT, 3) >> 4;
  //  handle negative values (20 bit)
  if (value & 0x00080000)
  {
    value |= 0xFFF00000;
  }
  float current = value * _current_LSB;
  return current;
}

//  PAGE 26 + 8.1.2
float Ina228::getPower()
{
  uint32_t value = _readRegister(INA228_POWER, 3);
  //  PAGE 31 (8.1.2)
  return value * 3.2 * _current_LSB;
}

//  PAGE 25
float Ina228::getTemperature()
{
  uint32_t value = _readRegister(INA228_TEMPERATURE, 2);
  float LSB = 7.8125e-3;  //  milli degree Celsius
  return value * LSB;
}

//  PAGE 26 + 8.1.2
double Ina228::getEnergy()
{
  //  read 40 bit UNSIGNED as a double to prevent 64 bit integers
  //  double might be 8 or 4 byte, depends on platform
  //  40 bit ==> O(10^12)
  double value = _readRegisterF(INA228_ENERGY, 'U');
  //  PAGE 31 (8.1.2)
  return value * (16 * 3.2) * _current_LSB;
}


//  PAGE 26 + 8.1.2
double Ina228::getCharge()
{
  //  read 40 bit SIGNED as a float to prevent 64 bit integers
  //  double might be 8 or 4 byte, depends on platform
  //  40 bit ==> O(10^12)
  double value = _readRegisterF(INA228_CHARGE, 'S');
  //  PAGE 32 (8.1.2)
  return value * _current_LSB;
}


////////////////////////////////////////////////////////
//
//  CONFIG REGISTER 0
//
void Ina228::reset()
{
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  value |= INA228_CFG_RST;
  _writeRegister(INA228_CONFIG, value);
}

bool Ina228::setAccumulation(uint8_t value)
{
  if (value > 1) return false;
  uint16_t reg = _readRegister(INA228_CONFIG, 2);
  if (value == 1) reg |= INA228_CFG_RSTACC;
  else            reg &= ~INA228_CFG_RSTACC;
  _writeRegister(INA228_CONFIG, reg);
  return true;
}

bool Ina228::getAccumulation()
{
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  return (value & INA228_CFG_RSTACC) > 0;
}

void Ina228::setConversionDelay(uint8_t steps)
{
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  value &= ~INA228_CFG_CONVDLY;
  value |= (steps << 6);
  _writeRegister(INA228_CONFIG, value);
}

uint8_t Ina228::getConversionDelay()
{
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  return (value >> 6) & 0xFF;
}

void Ina228::setTemperatureCompensation(bool on)
{
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  if (on) value |= INA228_CFG_TEMPCOMP;
  else    value &= ~INA228_CFG_TEMPCOMP;
  _writeRegister(INA228_CONFIG, value);
}

bool Ina228::getTemperatureCompensation()
{
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  return (value & INA228_CFG_TEMPCOMP) > 0;
}

bool Ina228::setADCRange(bool flag)
{
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  _ADCRange = (value & INA228_CFG_ADCRANGE) > 0;
  //  nothing changed ==> we're done.
  if (flag == _ADCRange) return true;

  _ADCRange = flag;
  if (flag) value |= INA228_CFG_ADCRANGE;
  else      value &= ~INA228_CFG_ADCRANGE;
  _writeRegister(INA228_CONFIG, value);
  //  Fix #26, issue where shunt_cal was not modified
  bool rv = setMaxCurrentShunt(getMaxCurrent(), getShunt()) == 0;
  return rv;
}

bool Ina228::getADCRange()
{
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  _ADCRange = (value & INA228_CFG_ADCRANGE) > 0;
  return _ADCRange;
}


////////////////////////////////////////////////////////
//
//  CONFIG ADC REGISTER 1
//
bool Ina228::setMode(uint8_t mode)
{
  if (mode > 0x0F) return false;
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  value &= ~INA228_ADC_MODE;
  value |= (mode << 12);
  _writeRegister(INA228_ADC_CONFIG, value);
  return true;
}

uint8_t Ina228::getMode()
{
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  return (value & INA228_ADC_MODE) >> 12;
}

bool Ina228::setBusVoltageConversionTime(uint8_t bvct)
{
  if (bvct > 7) return false;
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  value &= ~INA228_ADC_VBUSCT;
  value |= (bvct << 9);
  _writeRegister(INA228_ADC_CONFIG, value);
  return true;
}

uint8_t Ina228::getBusVoltageConversionTime()
{
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  return (value & INA228_ADC_VBUSCT) >> 9;
}

bool Ina228::setShuntVoltageConversionTime(uint8_t svct)
{
  if (svct > 7) return false;
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  value &= ~INA228_ADC_VSHCT;
  value |= (svct << 6);
  _writeRegister(INA228_ADC_CONFIG, value);
  return true;
}

uint8_t Ina228::getShuntVoltageConversionTime()
{
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  return (value & INA228_ADC_VSHCT) >> 6;
}

bool Ina228::setTemperatureConversionTime(uint8_t tct)
{
  if (tct > 7) return false;
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  value &= ~INA228_ADC_VTCT;
  value |= (tct << 3);
  _writeRegister(INA228_ADC_CONFIG, value);
  return true;
}

uint8_t Ina228::getTemperatureConversionTime()
{
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  return (value & INA228_ADC_VTCT) >> 3;
}

bool Ina228::setAverage(uint8_t avg)
{
  if (avg > 7) return false;
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  value &= ~INA228_ADC_AVG;
  value |= avg;
  _writeRegister(INA228_ADC_CONFIG, value);
  return true;
}

uint8_t Ina228::getAverage()
{
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  return (value & INA228_ADC_AVG);
}


////////////////////////////////////////////////////////
//
//  SHUNT CALIBRATION REGISTER 2
//
int Ina228::setMaxCurrentShunt(float maxCurrent, float shunt)
{
  //  Shunt can be really small
  if (shunt < 0.0001) return -2;    //  TODO error code
  if (maxCurrent < 0.0) return -3;  //  TODO error code
  _maxCurrent = maxCurrent;
  _shunt = shunt;
  _current_LSB = _maxCurrent * 1.9073486328125e-6;  //  pow(2, -19);

  //  PAGE 31 (8.1.2)
  float shunt_cal = 13107.2e6 * _current_LSB * _shunt;
  //  depends on ADCRANGE in INA228_CONFIG register.
  if (_ADCRange == true)
  {
    shunt_cal *= 4;
  }
  //  shunt_cal must be written to its REGISTER.
  _writeRegister(INA228_SHUNT_CAL, shunt_cal);

  return 0;
}

float Ina228::getMaxCurrent()
{
  return _maxCurrent;
}

float Ina228::getShunt()
{
  return _shunt;
}

float Ina228::getCurrentLSB()
{
  return _current_LSB;
}


////////////////////////////////////////////////////////
//
//  SHUNT TEMPERATURE COEFFICIENT REGISTER 3
//
bool Ina228::setShuntTemperatureCoefficent(uint16_t ppm)
{
  if (ppm > 16383) return false;
  _writeRegister(INA228_SHUNT_TEMP_CO, ppm);
  return true;
}

uint16_t Ina228::getShuntTemperatureCoefficent()
{
  uint16_t value = _readRegister(INA228_SHUNT_TEMP_CO, 2);
  return value;
}


////////////////////////////////////////////////////////
//
//  DIAGNOSE ALERT REGISTER 11
//
void Ina228::setDiagnoseAlert(uint16_t flags)
{
  _writeRegister(INA228_DIAG_ALERT, flags);
}

uint16_t Ina228::getDiagnoseAlert()
{
  return _readRegister(INA228_DIAG_ALERT, 2);
}

//  INA228.h has an enum for the bit fields.
void Ina228::setDiagnoseAlertBit(uint8_t bit)
{
  uint16_t value = _readRegister(INA228_DIAG_ALERT, 2);
  uint16_t mask = (1 << bit);
  //  only write new value if bit not set
  if ((value & mask) == 0)
  {
    value |= mask;
    _writeRegister(INA228_DIAG_ALERT, value);
  }
}

void Ina228::clearDiagnoseAlertBit(uint8_t bit)
{
  uint16_t value = _readRegister(INA228_DIAG_ALERT, 2);
  uint16_t mask = (1 << bit);
  //  only write new value if bit not set.
  if ((value & mask ) != 0)
  {
    value &= ~mask;
    _writeRegister(INA228_DIAG_ALERT, value);
  }
}

uint16_t Ina228::getDiagnoseAlertBit(uint8_t bit)
{
  uint16_t value = _readRegister(INA228_DIAG_ALERT, 2);
  return (value >> bit) & 0x01;
}


////////////////////////////////////////////////////////
//
//  THRESHOLD AND LIMIT REGISTERS 12-17
//
//  TODO  (sync INA228)
//  - API ?
//  - return bool for setters
//  - float voltage interface instead of uint16_t?  breaking!
void Ina228::setShuntOvervoltageTH(uint16_t threshold)
{
  //  TODO ADCRANGE DEPENDENT
  //  Conversion Factor: 5 μV/LSB when ADCRANGE = 0
  //  1.25 μV/LSB when ADCRANGE = 1.
  //  float LSB = 5.0e-6;
  //  if (_ADCRange == 1) LSB = 1.25e-6;
  _writeRegister(INA228_SOVL, threshold);
}

uint16_t Ina228::getShuntOvervoltageTH()
{
  //  TODO ADCRANGE DEPENDENT
  //  float LSB = 5.0e-6;
  //  if (_ADCRange == 1) LSB = 1.25e-6;
  return _readRegister(INA228_SOVL, 2);
}

void Ina228::setShuntUndervoltageTH(uint16_t threshold)
{
  //  TODO ADCRANGE DEPENDENT
  //  float LSB = 5.0e-6;
  //  if (_ADCRange == 1) LSB = 1.25e-6;
  _writeRegister(INA228_SUVL, threshold);
}

uint16_t Ina228::getShuntUndervoltageTH()
{
  //  TODO ADCRANGE DEPENDENT
  //  float LSB = 5.0e-6;
  //  if (_ADCRange == 1) LSB = 1.25e-6;
  return _readRegister(INA228_SUVL, 2);
}

void Ina228::setBusOvervoltageTH(uint16_t threshold)
{
  if (threshold > 0x7FFF) return;  //  false;
  //float LSB = 3.125e-3;  //  3.125 mV/LSB.
  _writeRegister(INA228_BOVL, threshold);
}

uint16_t Ina228::getBusOvervoltageTH()
{
  //float LSB = 3.125e-3;  //  3.125 mV/LSB.
  return _readRegister(INA228_BOVL, 2);
}

void Ina228::setBusUndervoltageTH(uint16_t threshold)
{
  if (threshold > 0x7FFF) return;
  //float LSB = 3.125e-3;  //  3.125 mV/LSB.
  _writeRegister(INA228_BUVL, threshold);
}

uint16_t Ina228::getBusUndervoltageTH()
{
  //float LSB = 3.125e-3;  //  3.125 mV/LSB.
  return _readRegister(INA228_BUVL, 2);
}

void Ina228::setTemperatureOverLimitTH(uint16_t threshold)
{
  //float LSB = 7.8125e-3;  //  milli degrees Celsius
  _writeRegister(INA228_TEMP_LIMIT, threshold);
}

uint16_t Ina228::getTemperatureOverLimitTH()
{
  //float LSB = 7.8125e-3;  //  milli degrees Celsius
  return _readRegister(INA228_TEMP_LIMIT, 2);
}

void Ina228::setPowerOverLimitTH(uint16_t threshold)
{
  //  P29
  //  Conversion factor: 256 × Power LSB.
  _writeRegister(INA228_POWER_LIMIT, threshold);
}

uint16_t Ina228::getPowerOverLimitTH()
{
  //  P29
  //  Conversion factor: 256 × Power LSB.
  return _readRegister(INA228_POWER_LIMIT, 2);
}


////////////////////////////////////////////////////////
//
//  MANUFACTURER and ID REGISTER 3E/3F
//
uint16_t Ina228::getManufacturer()
{
  uint16_t value = _readRegister(INA228_MANUFACTURER, 2);
  return value;
}

uint16_t Ina228::getDieID()
{
  uint16_t value = _readRegister(INA228_DEVICE_ID, 2);
  return (value >> 4) & 0x0FFF;
}

uint16_t Ina228::getRevision()
{
  uint16_t value = _readRegister(INA228_DEVICE_ID, 2);
  return value & 0x000F;
}


////////////////////////////////////////////////////////
//
//  ERROR HANDLING
//
int Ina228::getLastError()
{
  int e = _error;
  _error = 0;
  return e;
}


////////////////////////////////////////////////////////
//
//  PRIVATE
//
uint32_t Ina228::_readRegister(uint8_t reg, uint8_t bytes)
{
    _error = 0;

    uint8_t buffer[4] = {0};  // max expected
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(
        _i2c_handle,
        _address << 1,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buffer,
        bytes,
        100
    );

    if (status != HAL_OK)
    {
        _error = -1;
        return 0;
    }

    uint32_t value = 0;
    for (int i = 0; i < bytes; i++)
    {
        value <<= 8;
        value |= buffer[i];
    }

    return value;
}

//  always 5 bytes
double Ina228::_readRegisterF(uint8_t reg, char mode)
{
    _error = 0;

    uint8_t buffer[5] = {0};
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(
        _i2c_handle,
        _address << 1,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buffer,
        5,
        100
    );

    if (status != HAL_OK)
    {
        _error = -1;
        return 0;
    }

    uint32_t val = 0;

    for (int i = 0; i < 4; i++)
    {
        val <<= 8;
        val |= buffer[i];
    }

    double value;
    if (mode == 'U') value = (double)val;
    else             value = (double)((int32_t)val);

    value *= 256.0;
    value += buffer[4];

    return value;
}


uint16_t Ina228::_writeRegister(uint8_t reg, uint16_t value)
{
    uint8_t buffer[2];
    buffer[0] = (value >> 8) & 0xFF;
    buffer[1] = value & 0xFF;

    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write(
        _i2c_handle,
        _address << 1,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buffer,
        2,
        100
    );

    if (status != HAL_OK)
    {
        _error = -1;
        return 1;  // non-zero = error (Arduino-compatible behavior)
    }

    return 0;
}


//  -- END OF FILE --

