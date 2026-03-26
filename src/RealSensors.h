#pragma once

#include "Sensors.h"
#include <Wire.h>
#include "MTi.h"
#include "Measurements.h"
#include <Adafruit_BMP3XX.h>
#include "pins.h"

#define REAL_SENSORS_FLAG 1

extern MTi *imu;
extern float groundAltitude;

class RealSensors : public SensorBackend {
    public:
        bool initializeIMU() override;
        bool performIMUTare() override;
        bool readIMU(IMU_Measurements& imu_meas) override;

        bool initializeBarometer() override;
        bool performBarometerTare() override;
        bool readBarometer(BARO_Measurements& baro_meas) override;

        float sil_true_alt() override;
    };
