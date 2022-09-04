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

#include "globals.h"

#include "mpu6050sensor.h"
#include "network/network.h"
#include <i2cscan.h>
#include "calibration.h"
#include "GlobalVars.h"
#include "vqf.h"

void MPU6050Sensor::motionSetup()
{
    imu.initialize(addr);
    if (!imu.testConnection())
    {
        m_Logger.fatal("Can't connect to MPU6050 (reported device ID 0x%02x) at address 0x%02x", imu.getDeviceID(), addr);
        return;
    }

    m_Logger.info("Connected to MPU6050 (reported device ID 0x%02x) at address 0x%02x", imu.getDeviceID(), addr);

    working = true;

    // Flip imu upside down, turn on, wait 5 seconds, then flip back up to start callibration
    // if (isUpsideDownAndFlippedBack)
    // {
    //     startCalibration(0);
    // }

    // Initialize the configuration
    {
        SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
        // If no compatible calibration data is found, the calibration data will just be zero-ed out
        switch (sensorCalibration.type)
        {
        case SlimeVR::Configuration::CalibrationConfigType::MPU6050:
            m_Calibration = sensorCalibration.data.mpu6050;
            // Setting the gyro offset from calibration
            imu.setXGyroOffset(m_Calibration.G_off[0]);
            imu.setYGyroOffset(m_Calibration.G_off[1]);
            imu.setZGyroOffset(m_Calibration.G_off[2]);
            // Setting the accel bias from calibration
            imu.setXAccelOffset(m_Calibration.A_B[0]);
            imu.setYAccelOffset(m_Calibration.A_B[1]);
            imu.setZAccelOffset(m_Calibration.A_B[2]);
            break;

        case SlimeVR::Configuration::CalibrationConfigType::NONE:
            m_Logger.warn("No calibration data found for sensor %d, ignoring...", sensorId);
            m_Logger.info("Calibration is advised");
            break;

        default:
            m_Logger.warn("Incompatible calibration data found for sensor %d, ignoring...", sensorId);
            m_Logger.info("Calibration is advised");
        }
    }

    //Enable digital low pass filter. Gyro and accel now output at 1Khz. Not sure yet which mode would be best. Higher would mean better low pass filter, but also higher delay. max = 6, off = 0 (at 0 the gyro can update at 8khz.)
    imu.setDLPFMode(2);

    // Enable FIFO, and gyro + accel to write to FIFO
    imu.setFIFOEnabled(true);
    imu.setXGyroFIFOEnabled(true);
    imu.setYGyroFIFOEnabled(true);
    imu.setZGyroFIFOEnabled(true);
    imu.setAccelFIFOEnabled(true);
    imu.setRate(10);

    // Setup VQF filter
    // vqf = vqf.VQF(1 / 8000, 1 / 1000, 0);

    configured = true;
}

void MPU6050Sensor::motionLoop()
{
    uint16_t fifoCount = imu.getFIFOCount();
    uint16_t packetCount = fifoCount / FIFO_PACKET_SIZE;
    m_Logger.debug("FIFO size: %d", fifoCount);


    //Fifo buffer is full, assuming overflow and resetting...
    if(fifoCount == sizeof(fifoPacket))
    { 
        m_Logger.info("FIFO buffer is full, assuming overflow and resetting....");
        imu.resetFIFO();
        return;
    }

    if(fifoCount < FIFO_PACKET_SIZE) return; //Nothing to read yet in the fifo buffer

    uint16_t ax, ay,az, gx, gy, gz;
    imu.getFIFOBytes(fifoPacket, fifoCount);
    m_Logger.debug("currently %d packets in the fifo", packetCount);

    for(uint16_t i = 0; i < fifoCount;)
    {
        ax = (int16_t)fifoPacket[i]  << 8  | (int16_t)fifoPacket[i + 1];
        ay = (int16_t)fifoPacket[i +2]  << 8  | (int16_t)fifoPacket[i + 3];
        az = (int16_t)fifoPacket[i + 4]  << 8  | (int16_t)fifoPacket[i + 5];
        gx = (int16_t)fifoPacket[i + 6]  << 8  | (int16_t)fifoPacket[i + 7];
        gy = (int16_t)fifoPacket[i + 8]  << 8  | (int16_t)fifoPacket[i + 9];
        gz = (int16_t)fifoPacket[i + 10] << 8  | (int16_t)fifoPacket[i + 11];
        m_Logger.debug("Read FIFO packet. ax:%d ay:%d az:%d gx:%d gy:%d gz:%d", ax, ay, az, gx, gy,gz);
        i+=FIFO_PACKET_SIZE;

        // if (!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion))
        // {
        //     newData = true;
        //     lastQuatSent = quaternion;
        // }

    }


}

void MPU6050Sensor::startCalibration(int calibrationType)
{
    ledManager.on();

    m_Logger.info("Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    ledManager.off();
    // imu.setDMPEnabled(false);
    imu.CalibrateGyro(6);
    imu.CalibrateAccel(6);
    // imu.setDMPEnabled(true);

    m_Logger.debug("Gathered baseline gyro reading");
    ledManager.blink(50);
    ledManager.blink(50);
    m_Logger.debug("Starting offset finder");

    // Calibrating accel
    imu.CalibrateAccel(10);
    m_Calibration.A_B[0] = imu.getXAccelOffset();
    m_Calibration.A_B[1] = imu.getYAccelOffset();
    m_Calibration.A_B[2] = imu.getZAccelOffset();

    ledManager.blink(50);
    ledManager.blink(50);

    // Calibrating gyro
    imu.CalibrateGyro(10);
    m_Calibration.G_off[0] = imu.getXGyroOffset();
    m_Calibration.G_off[1] = imu.getYGyroOffset();
    m_Calibration.G_off[2] = imu.getZGyroOffset();

    ledManager.blink(50);
    ledManager.blink(50);
    ledManager.blink(50);
    ledManager.blink(50);
    SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = SlimeVR::Configuration::CalibrationConfigType::MPU6050;
    calibration.data.mpu6050 = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();

    ledManager.blink(50);
    ledManager.blink(50);
    ledManager.blink(50);
    ledManager.blink(50);

    m_Logger.info("Calibration finished");

    ledManager.off();
}

bool MPU6050Sensor::isUpsideDownAndFlippedBack()
{

    // turn on while flip back to calibrate. then, flip again after 5 seconds.
    int16_t ax, ay, az;
    imu.getAcceleration(&ax, &ay, &az);
    float g_az = (float)az / TYPICAL_ACCEL_SENSITIVITY; // For 2G sensitivity
    if (g_az < -0.75f)
    {
        ledManager.on();
        m_Logger.info("Flip front to confirm start calibration");
        delay(5000);
        ledManager.off();

        imu.getAcceleration(&ax, &ay, &az);
        g_az = (float)az / TYPICAL_ACCEL_SENSITIVITY;
        if (g_az > 0.75f)
        {
            return true;
        }
    }
    return false;
}
