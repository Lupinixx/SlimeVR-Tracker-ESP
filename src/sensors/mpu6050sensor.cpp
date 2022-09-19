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


#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "mpu6050sensor.h"

// Packet size of the fifo packets and total fifo size.
#define FIFO_PACKET_SIZE 28
#define FIFO_BUFFER_SIZE 1024

// 131 LSB/deg/s = 250 deg/s
#define TYPICAL_GYRO_SENSITIVITY 131
// 16384 LSB/G = 2G
#define TYPICAL_ACCEL_SENSITIVITY 16384.

// Gyro scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr float GSCALE = ((32768. / TYPICAL_GYRO_SENSITIVITY) / 32768.) * (PI / 180.0);
// Accel scale conversion steps: LSB/G -> G -> m/s^2
constexpr float ASCALE = ((32768. / TYPICAL_ACCEL_SENSITIVITY) / 32768.) * SENSORS_GRAVITY_EARTH;

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

    int16_t ax,ay,az;

    // turn on while flip back to calibrate. then, flip again after 5 seconds.
    // TODO: Move calibration invoke after calibrate button on slimeVR server available
    imu.getAcceleration(&ax, &ay, &az);
    float g_az = (float)az / TYPICAL_ACCEL_SENSITIVITY; // For 2G sensitivity
    if(g_az < -0.75f) {
        ledManager.on();
        m_Logger.info("Flip front to confirm start calibration");
        delay(5000);
        ledManager.off();

        imu.getAcceleration(&ax, &ay, &az);
        g_az = (float)az / TYPICAL_ACCEL_SENSITIVITY;
        if(g_az > 0.75f) {
            m_Logger.debug("Starting calibration...");
            startCalibration(0);
        }
    }

    // Initialize the configuration
    {
        SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
        // If no compatible calibration data is found, the calibration data will just be zero-ed out
        switch (sensorCalibration.type) {
        case SlimeVR::Configuration::CalibrationConfigType::MPU9250:
            m_Calibration = sensorCalibration.data.mpu9250;
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

    // turn on the DMP, now that it's ready
    m_Logger.debug("Enabling DMP...");
    imu.dmpInitialize();
    imu.setDMPEnabled(true);

    configured = true;
}

void MPU6050Sensor::motionLoop()
{
    uint16_t fifoCount = imu.getFIFOCount();
    // m_Logger.debug("FIFO size: %d", fifoCount);

    // Fifo buffer is full, assuming overflow and resetting...
    if (fifoCount == FIFO_BUFFER_SIZE)
    {
        m_Logger.info("FIFO buffer is full, assuming overflow and resetting....");
        imu.resetFIFO();
        return;
    }

    if (fifoCount < FIFO_PACKET_SIZE)
        return; // Nothing to read yet in the fifo buffer

    int16_t ax, ay, az, gx, gy, gz;
    // m_Logger.debug("currently %d packets in the fifo", packetCount);

    for (uint16_t outerI = 0; outerI < fifoCount;)
    {
        uint8_t fifoAmountToGet = (fifoCount > 120) ? 120 : fifoCount;
        // m_Logger.debug("processing %d packets", fifoAmountToGet / 12);
        imu.getFIFOBytes(fifoPacket, fifoAmountToGet);

        for (uint8_t innerI = 0; innerI < fifoAmountToGet;)
        {
            ax = (int16_t)fifoPacket[innerI + 22] << 8 | (int16_t)fifoPacket[innerI + 23];
            ay = (int16_t)fifoPacket[innerI + 24] << 8 | (int16_t)fifoPacket[innerI + 25];
            az = (int16_t)fifoPacket[innerI + 26] << 8 | (int16_t)fifoPacket[innerI + 27];
            gx = (int16_t)fifoPacket[innerI + 16] << 8 | (int16_t)fifoPacket[innerI + 17];
            gy = (int16_t)fifoPacket[innerI + 18] << 8 | (int16_t)fifoPacket[innerI + 19];
            gz = (int16_t)fifoPacket[innerI + 20] << 8 | (int16_t)fifoPacket[innerI + 21];
            // m_Logger.debug("Read FIFO packet. ax:%d ay:%d az:%d gx:%d gy:%d gz:%d", ax, ay, az, gx, gy, gz);

            Gxyz[0] = (float)gx * GSCALE; // 250 LSB(d/s) default to radians/s
            Gxyz[1] = (float)gy * GSCALE;
            Gxyz[2] = (float)gz * GSCALE;

            Axyz[0] = (float)ax * ASCALE;
            Axyz[1] = (float)ay * ASCALE;
            Axyz[2] = (float)az * ASCALE;

            vqf.update(Gxyz, Axyz);

            innerI += FIFO_PACKET_SIZE;
        }
        outerI += fifoAmountToGet;
    }
    vqf.getQuat6D(q);
    quaternion.set(-q[2], q[1], q[3], q[0]);
    quaternion *= sensorOffset;

    if (!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion))
    {
        m_Logger.debug("Sending new data");
        newData = true;
        lastQuatSent = quaternion;
    }
}

void MPU6050Sensor::startCalibration(int calibrationType)
{
    ledManager.on();
    m_Logger.debug("Gathering raw data for device calibration...");
    constexpr uint16_t calibrationSamples = 300;
    // Reset values
    Gxyz[0] = 0;
    Gxyz[1] = 0;
    Gxyz[2] = 0;

    // Wait for sensor to calm down before calibration
    m_Logger.info("Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    for (int i = 0; i < calibrationSamples; i++)
    {
        int16_t gx, gy, gz;
        imu.getRotation(&gx, &gy, &gz);
        Gxyz[0] += float(gx);
        Gxyz[1] += float(gy);
        Gxyz[2] += float(gz);
    }
    Gxyz[0] /= calibrationSamples;
    Gxyz[1] /= calibrationSamples;
    Gxyz[2] /= calibrationSamples;

#ifdef DEBUG_SENSOR
    m_Logger.trace("Gyro calibration results: %f %f %f", Gxyz[0], Gxyz[1], Gxyz[2]);
#endif

    Network::sendRawCalibrationData(Gxyz, CALIBRATION_TYPE_EXTERNAL_GYRO, 0);
    m_Calibration.G_off[0] = Gxyz[0];
    m_Calibration.G_off[1] = Gxyz[1];
    m_Calibration.G_off[2] = Gxyz[2];

    // Blink calibrating led before user should rotate the sensor
    m_Logger.info("Gently rotate the device while it's gathering accelerometer and magnetometer data");
    ledManager.pattern(15, 300, 3000 / 310);
    float *calibrationDataAcc = (float *)malloc(calibrationSamples * 3 * sizeof(float));
    for (int i = 0; i < calibrationSamples; i++)
    {
        ledManager.on();
        int16_t ax, ay, az;
        imu.getAcceleration(&ax, &ay, &az);
        calibrationDataAcc[i * 3 + 0] = ax;
        calibrationDataAcc[i * 3 + 1] = ay;
        calibrationDataAcc[i * 3 + 2] = az;
        Network::sendRawCalibrationData(calibrationDataAcc, CALIBRATION_TYPE_EXTERNAL_ACCEL, 0);
        ledManager.off();
        delay(250);
    }
    m_Logger.debug("Calculating calibration data...");

    float A_BAinv[4][3];
    CalculateCalibration(calibrationDataAcc, calibrationSamples, A_BAinv);
    free(calibrationDataAcc);
    m_Logger.debug("Finished Calculate Calibration data");
    m_Logger.debug("Accelerometer calibration matrix:");
    m_Logger.debug("{");
    for (uint8_t i = 0; i < 3; i++)
    {
        m_Calibration.A_B[i] = A_BAinv[0][i];
        m_Calibration.A_Ainv[0][i] = A_BAinv[1][i];
        m_Calibration.A_Ainv[1][i] = A_BAinv[2][i];
        m_Calibration.A_Ainv[2][i] = A_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", A_BAinv[0][i], A_BAinv[1][i], A_BAinv[2][i], A_BAinv[3][i]);
    }
    m_Logger.debug("}");

    m_Logger.debug("Saving the calibration data");

    SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = SlimeVR::Configuration::CalibrationConfigType::MPU6050;
    calibration.data.mpu6050 = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();

    ledManager.off();
    Network::sendCalibrationFinished(CALIBRATION_TYPE_EXTERNAL_ALL, 0);
    m_Logger.debug("Saved the calibration data");

    m_Logger.info("Calibration data gathered");
}
