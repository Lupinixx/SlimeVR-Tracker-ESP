/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#ifndef SENSORS_MPU6050SENSOR_H
#define SENSORS_MPU6050SENSOR_H

#include "sensor.h"
#include <MPU6050.h>
#include "vqf.h"

// 131 LSB/deg/s = 250 deg/s
#define TYPICAL_GYRO_SENSITIVITY 131
// 16384 LSB/G = 2G
#define TYPICAL_ACCEL_SENSITIVITY 16384.

// Packet size of the fifo packets. This is dependent on the
#define FIFO_PACKET_SIZE 12
#define FIFO_BUFFER_SIZE 1024

// Gyro scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr float GSCALE = ((32768. / TYPICAL_GYRO_SENSITIVITY) / 32768.) * (PI / 180.0);
// Accel scale conversion steps: LSB/G -> G -> m/s^2
constexpr float ASCALE = ((32768. / TYPICAL_ACCEL_SENSITIVITY) / 32768.) * SENSORS_GRAVITY_EARTH;

class MPU6050Sensor : public Sensor
{
public:
    MPU6050Sensor(uint8_t id, uint8_t type, uint8_t address, float rotation) : Sensor("MPU6050Sensor", type, id, address, rotation){};
    ~MPU6050Sensor(){};
    void motionSetup() override final;
    void motionLoop() override final;
    void startCalibration(int calibrationType) override final;
    bool isUpsideDownAndFlippedBack();

private:
    MPU6050 imu{};
    uint8_t fifoPacket[120];            // Buffer to store whole mpu fifo buffer in to proccess. 120 bytes because that seems to be the max for a burst read for the current packet size. 124 bytes seems to be max.
    float q[4]{1.0f, 0.0f, 0.0f, 0.0f}; // for raw filter
    float Axyz[3]{};
    float Gxyz[3]{};
    float Mxyz[3]{};
    uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
    uint8_t devStatus;    // return status after each device operation (0 = success, !0 = error)
    //Timing variables
    uint8_t sampleRateDevicer = 1;
    VQF vqf{(float)(sampleRateDevicer + 1) / 1000};

    SlimeVR::Configuration::MPU6050CalibrationConfig m_Calibration;
};

#endif
