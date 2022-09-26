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

#include "mpu6050sensor.h"

// Packet size of the fifo packets and total fifo size.
#define FIFO_PACKET_SIZE 12
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

    // Enable digital low pass filter. Gyro and accel now output at 1Khz. Not sure yet which mode would be best. Higher would mean better low pass filter, but also higher delay. max = 6, off = 0 (at 0 the gyro can update at 8khz.)
    imu.setDLPFMode(2);
    // set configured sample rate.
    imu.setRate(sampleRateDevider - 1);

    int16_t ax, ay, az;
    // turn on while flip back to calibrate. then, flip again after 5 seconds.
    // TODO: Move calibration invoke after calibrate button on slimeVR server available
    imu.getAcceleration(&ax, &ay, &az);
    float g_az = (float)az / TYPICAL_ACCEL_SENSITIVITY;
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
            m_Logger.debug("Starting calibration...");
            startCalibration(0);
        }
    }

    // Initialize the configuration
    {
        SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
        // If no compatible calibration data is found, the calibration data will just be zero-ed out
        switch (sensorCalibration.type)
        {
        case SlimeVR::Configuration::CalibrationConfigType::MPU6050:

            // m_Calibration = sensorCalibration.data.mpu6050;
            // // Setting the gyro offset from calibration
            // imu.setXGyroOffset(m_Calibration.G_off[0]);
            // imu.setYGyroOffset(m_Calibration.G_off[1]);
            // imu.setZGyroOffset(m_Calibration.G_off[2]);
            // // Setting the accel bias from calibration
            // imu.setXAccelOffset(m_Calibration.A_B[0]);
            // imu.setYAccelOffset(m_Calibration.A_B[1]);
            // imu.setZAccelOffset(m_Calibration.A_B[2]);
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

    // Enable FIFO, and gyro + accel to write to FIFO
    imu.setFIFOEnabled(true);
    imu.setXGyroFIFOEnabled(true);
    imu.setYGyroFIFOEnabled(true);
    imu.setZGyroFIFOEnabled(true);
    imu.setAccelFIFOEnabled(true);

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
            ax = (int16_t)fifoPacket[innerI] << 8 | (int16_t)fifoPacket[innerI + 1];
            ay = (int16_t)fifoPacket[innerI + 2] << 8 | (int16_t)fifoPacket[innerI + 3];
            az = (int16_t)fifoPacket[innerI + 4] << 8 | (int16_t)fifoPacket[innerI + 5];
            gx = (int16_t)fifoPacket[innerI + 6] << 8 | (int16_t)fifoPacket[innerI + 7];
            gy = (int16_t)fifoPacket[innerI + 8] << 8 | (int16_t)fifoPacket[innerI + 9];
            gz = (int16_t)fifoPacket[innerI + 10] << 8 | (int16_t)fifoPacket[innerI + 11];
            // m_Logger.debug("Read FIFO packet. ax:%d ay:%d az:%d       gx:%d gy:%d gz:%d", ax, ay, az, gx, gy, gz);

            Gxyz[0] = ((float)gx - m_Calibration.G_off[0]) * GSCALE;
            Gxyz[1] = ((float)gy - m_Calibration.G_off[1]) * GSCALE;
            Gxyz[2] = ((float)gz - m_Calibration.G_off[2]) * GSCALE;

            float temp[3];
            int i;

            for (i = 0; i < 3; i++)
                temp[i] = (Axyz[i] - m_Calibration.A_B[i]);
            Axyz[0] = m_Calibration.A_Ainv[0][0] * temp[0] + m_Calibration.A_Ainv[0][1] * temp[1] + m_Calibration.A_Ainv[0][2] * temp[2];
            Axyz[1] = m_Calibration.A_Ainv[1][0] * temp[0] + m_Calibration.A_Ainv[1][1] * temp[1] + m_Calibration.A_Ainv[1][2] * temp[2];
            Axyz[2] = m_Calibration.A_Ainv[2][0] * temp[0] + m_Calibration.A_Ainv[2][1] * temp[1] + m_Calibration.A_Ainv[2][2] * temp[2];

            Axyz[0] = (float)ax * ASCALE;
            Axyz[1] = (float)ay * ASCALE;
            Axyz[2] = (float)az * ASCALE;

            vqf.update(Gxyz, Axyz);
            // m_Logger.debug("Calibrated values: ax:%f ay:%f az:%f       gx:%f gy:%f gz:%f", Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2]);

            innerI += FIFO_PACKET_SIZE;
        }
        outerI += fifoAmountToGet;
    }
    vqf.getQuat6D(q);
    quaternion.set(-q[2], q[1], q[3], q[0]);
    quaternion *= sensorOffset;

    if (!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion))
    {
        newData = true;
        lastQuatSent = quaternion;
    }
}

// void MPU6050Sensor::startCalibration(int calibrationType)
// {
//     ledManager.on();

//     imu.setXGyroOffset(0);
//     imu.setYGyroOffset(0);
//     imu.setZGyroOffset(0);
//     imu.setXAccelOffset(0);
//     imu.setYAccelOffset(0);
//     imu.setZAccelOffset(0);

//     m_Logger.info("Put down the device and wait for baseline gyro reading calibration");
//     delay(2000);
//     ledManager.on();
//     imu.CalibrateGyro(20);
//     m_Calibration.G_off[0] = imu.getXGyroOffset();
//     m_Calibration.G_off[1] = imu.getYGyroOffset();
//     m_Calibration.G_off[2] = imu.getZGyroOffset();
//     m_Logger.info("");
//     m_Logger.info("Gyro calibration done. Offsets: X[%d] Y[%d] Z[%d]", m_Calibration.G_off[0], m_Calibration.G_off[1], m_Calibration.G_off[2]);
//     m_Logger.info("Starting baseline accel reading calibration");
//     imu.CalibrateAccel(20);
//     m_Calibration.A_B[0] = imu.getXAccelOffset();
//     m_Calibration.A_B[1] = imu.getYAccelOffset();
//     m_Calibration.A_B[2] = imu.getZAccelOffset();
//     ledManager.off();
//     m_Logger.info("Accel calibration done. Offsets: X[%d] Y[%d] Z[%d]", m_Calibration.A_B[0], m_Calibration.A_B[1], m_Calibration.A_B[2]);

//     ledManager.blink(50);
//     ledManager.blink(50);
//     ledManager.blink(50);
//     ledManager.blink(50);
//     SlimeVR::Configuration::CalibrationConfig calibration;
//     calibration.type = SlimeVR::Configuration::CalibrationConfigType::MPU6050;
//     calibration.data.mpu6050 = m_Calibration;
//     configuration.setCalibration(sensorId, calibration);
//     configuration.save();

//     ledManager.blink(50);
//     ledManager.blink(50);
//     ledManager.blink(50);
//     ledManager.blink(50);

//     m_Logger.info("Calibration finished");

//     ledManager.off();
// }

void MPU6050Sensor::startCalibration(int calibrationType)
{
    ledManager.on();

    m_Logger.debug("Entering calibration mode");

    // Wait for sensor to calm down before calibration
    constexpr uint8_t GYRO_CALIBRATION_DELAY_SEC = 3;
    uint16_t gyroCalibrationSamples = 300;
    m_Logger.info("Put down the device and wait for baseline gyro reading calibration (%i seconds)", GYRO_CALIBRATION_DELAY_SEC);
    ledManager.on();
    for (uint8_t i = GYRO_CALIBRATION_DELAY_SEC; i > 0; i--)
    {
        m_Logger.info("%i...", i);
        delay(1000);
    }
    ledManager.off();

    float rawGxyz[3] = {0};

    m_Logger.info("Gyro calibration started...");
    ledManager.on();
    for (int i = 0; i < gyroCalibrationSamples; i++)
    {
        int16_t gx, gy, gz;
        imu.getRotation(&gx, &gy, &gz);
        rawGxyz[0] += float(gx);
        rawGxyz[1] += float(gy);
        rawGxyz[2] += float(gz);
    }
    ledManager.off();
    m_Calibration.G_off[0] = rawGxyz[0] / gyroCalibrationSamples;
    m_Calibration.G_off[1] = rawGxyz[1] / gyroCalibrationSamples;
    m_Calibration.G_off[2] = rawGxyz[2] / gyroCalibrationSamples;

    m_Logger.debug("Gyro calibration results: %f %f %f", UNPACK_VECTOR_ARRAY(m_Calibration.G_off));

    // Blink calibrating led before user should rotate the sensor
    m_Logger.info("After 3 seconds, Gently rotate the device while it's gathering data");
    constexpr uint8_t ACCEL_CALIBRATION_DELAY_SEC = 3;
    ledManager.on();
    for (uint8_t i = ACCEL_CALIBRATION_DELAY_SEC; i > 0; i--)
    {
        m_Logger.info("%i...", i);
        delay(1000);
    }
    ledManager.off();
    m_Logger.debug("Gathering data...");

    uint16_t secondaryCalibrationSamples = 300;

    float *calibrationDataAcc = (float *)malloc(secondaryCalibrationSamples * 3 * sizeof(float));
    ledManager.on();
    for (int i = 0; i < secondaryCalibrationSamples; i++)
    {
        int16_t ax, ay, az;
        imu.getAcceleration(&ax, &ay, &az);
        calibrationDataAcc[i * 3 + 0] = ax;
        calibrationDataAcc[i * 3 + 1] = ay;
        calibrationDataAcc[i * 3 + 2] = az;

        delay(100);
    }
    ledManager.off();
    m_Logger.debug("Calculating calibration data...");

    float A_BAinv[4][3];
    CalculateCalibration(calibrationDataAcc, secondaryCalibrationSamples, A_BAinv);
    free(calibrationDataAcc);
    m_Logger.debug("Finished Calculate Calibration data");
    m_Logger.debug("Accelerometer calibration matrix:");
    m_Logger.debug("{");
    for (int i = 0; i < 3; i++)
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

    m_Logger.debug("Saved the calibration data");

    m_Logger.info("Calibration data gathered, exiting calibration mode in...");
    constexpr uint8_t POST_CALIBRATION_DELAY_SEC = 5;
    ledManager.on();
    for (uint8_t i = POST_CALIBRATION_DELAY_SEC; i > 0; i--)
    {
        m_Logger.info("%i...", i);
        delay(1000);
    }
}