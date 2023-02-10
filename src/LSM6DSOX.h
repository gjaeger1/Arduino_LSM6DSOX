#pragma once

/*
  This file is part of the Arduino_LSM6DSOX library.
  Copyright (c) 2021 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifdef ADMORPH_PLATFORM_IS_PICO
    #include "pico/binary_info.h"
    #include "hardware/i2c.h"
#endif

class LSM6DSOXClass {
  public:
    LSM6DSOXClass(uint8_t slaveAddress);
    ~LSM6DSOXClass();

    int begin();
    void end();

    // Accelerometer
    void setAccelerationFrequency(const int freq);
    int readAcceleration(float& x, float& y, float& z); // Results are in g (earth gravity).
    int accelerationSampleRate(); // Sampling rate of the sensor.
    int accelerationAvailable(); // Check for available data from accelerometer

    // Gyroscope
    void setGyroscopeFrequency(const int freq);
    int readGyroscope(float& x, float& y, float& z); // Results are in degrees/second.
    int gyroscopeSampleRate(); // Sampling rate of the sensor.
    int gyroscopeAvailable(); // Check for available data from gyroscope

    // Temperature
    int readTemperature(int& temperature_deg);
    int readTemperatureFloat(float& temperature_deg);
    int temperatureAvailable();

  private:
    int readRegister(uint8_t address);
    int readRegisters(uint8_t address, uint8_t* data, size_t length);
    int writeRegister(uint8_t address, uint8_t value);


  private:
    int getFrequencyBinary(const int hz) const;
  
    uint8_t _slaveAddress;
    int _accHz = 104;
    int _gyrHz = 104;
    i2c_inst_t* i2c_instance;
};

extern LSM6DSOXClass IMU_LSM6DSOX;
#undef IMU
#define IMU IMU_LSM6DSOX
