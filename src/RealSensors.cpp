
#include "RealSensors.h"
#include <Wire.h>
#include "MTi.h"
#include "Measurements.h"
#include <Adafruit_BMP3XX.h>

MTi *imu = nullptr;
Adafruit_BMP3XX baro;
float imuTare[6] = {0.0f};
bool imuTared = false;
float groundAltitude = 0.0f;


bool RealSensors::initializeIMU() {

    Wire2.begin();
    
    pinMode(IMU_DRDY_PIN, INPUT);
    // imu = new MTi(IMU_ADDRESS, IMU_DRDY_PIN);

    static MTi imu_obj(IMU_ADDRESS, IMU_DRDY_PIN);
    imu = &imu_obj;


    if (!imu->detect(1000)) {
        return false;
    }

    imu->goToConfig();
    delay(100);
    imu->requestDeviceInfo();
    delay(100);
    imu->configureOutputs();
    delay(100);
    imu->goToMeasurement();
    delay(100);
    delay(500);

    return true;
}

bool RealSensors::performIMUTare() {
    float sum[6] = {0};
    int samples = 0;
    unsigned long start = millis();

    while (millis() - start < 10000) {
        if (digitalRead(IMU_DRDY_PIN)) {
            imu->readMessages();
            float *acc  = imu->getAcceleration();
            float *gyro = imu->getRateOfTurn();

            sum[0] += acc[0];
            sum[1] += acc[1];
            sum[2] += acc[2];
            sum[3] += gyro[0];
            sum[4] += gyro[1];
            sum[5] += gyro[2];
            samples++;
        }
        delay(10);
    }

    if (samples > 0) {
        for (int i = 0; i < 6; i++) {
            imuTare[i] = sum[i] / samples;
        }
        imuTared = true;
        return true;
    }
    return false;
}

bool RealSensors::readIMU(IMU_Measurements &imu_meas) {
    if (!imu) return false;
    
    // Only read if data is ready
    if (!digitalRead(IMU_DRDY_PIN)) return false;
    
    imu->readMessages();
    
    float *acc  = imu->getAcceleration();
    float *gyro = imu->getRateOfTurn();
    float *eulerAngles = imu->getEulerAngles();

    if (imuTared) {
        // imu_meas.accelX = acc[0] - imuTare[0];
        // imu_meas.accelY = acc[1] - imuTare[1];
        // imu_meas.accelZ = acc[2] - imuTare[2];
        imu_meas.accelX = acc[0];
        imu_meas.accelY = acc[1];
        imu_meas.accelZ = acc[2];
        imu_meas.gyroX  = gyro[0] - imuTare[3];
        imu_meas.gyroY  = gyro[1] - imuTare[4];
        imu_meas.gyroZ  = gyro[2] - imuTare[5];
        imu_meas.roll_deg = eulerAngles[0];
        imu_meas.pitch_deg = eulerAngles[1];
        imu_meas.yaw_deg = eulerAngles[2];
    } else {
        imu_meas.accelX = acc[0];
        imu_meas.accelY = acc[1];
        imu_meas.accelZ = acc[2];
        imu_meas.gyroX  = gyro[0];
        imu_meas.gyroY  = gyro[1];
        imu_meas.gyroZ  = gyro[2];
        imu_meas.roll_deg = eulerAngles[0];
        imu_meas.pitch_deg = eulerAngles[1];
        imu_meas.yaw_deg = eulerAngles[2];
    }

    // Timer updates
    uint32_t now = micros();
    imu_meas.IMU_dt_us = now - imu_meas.last_IMU_reading_time_us;
    imu_meas.last_IMU_reading_time_us = now;

    return true;
}

bool RealSensors::initializeBarometer() {

    Wire2.begin();

    if (!baro.begin_I2C(0x77, &Wire2)) {
        return false;
    }

    baro.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    baro.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    baro.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
    baro.setOutputDataRate(BMP3_ODR_25_HZ);

    return true;
}

bool RealSensors::performBarometerTare() {
    float altSum = 0.0f;
    int altSamples = 0;

    const int DISCARD_READINGS = 10;
    const int TOTAL_READINGS   = 110;  // 10 discard + 100 used

    int validCount = 0;

    for (int i = 0; i < TOTAL_READINGS; i++) {
        if (baro.performReading()) {
            float alt = baro.readAltitude(1013.25);

            // Skip first few valid readings
            if (validCount >= DISCARD_READINGS) {
                altSum += alt;
                altSamples++;
            }

            validCount++;
        }
        delay(10);
    }  

    if (altSamples > 0) {
        groundAltitude = altSum / altSamples;
        Serial.print("\n Barometer Tare: ");
        Serial.println(groundAltitude);
        return true;
    } else {
        return false;
    }
}

bool RealSensors::readBarometer(BARO_Measurements &baro_meas) {

    baro_meas.baroAltitude    = baro.readAltitude(1013.25f);
    baro_meas.baroPressure    = baro.pressure / 100.0f;
    baro_meas.baroTemperature = baro.temperature;
    
    return true;
}

float RealSensors::sil_true_alt() {
    return 0.0;
}