#pragma once
#include "Arduino.h"
#include "SensorBackend.h"
#include "sil_data.h"

#define SIL_SENSORS_FLAG 1

extern float groundAltitude;

class SIL_Sensors : public SensorBackend {
public:
    float t = 0;
    float euler_rad[3];
    float accel_mps2[3];
    float gyro_rps[3];

    //fake MTI methods
    int findTimeIndexBinary(float t_runtime);
    void readMessages(float t_s);
    float* getEulerAngles();
    float* getAcceleration();
    float* getRateOfTurn();
    float readAltitude(float sea_level_pressure_pa, float t_s);

    float sil_true_alt() override;

    //Fake Sensors methods
    bool initializeIMU() override;
    bool performIMUTare() override;
    bool readIMU(IMU_Measurements& imu_meas) override;

    bool initializeBarometer() override;
    bool performBarometerTare() override;
    bool readBarometer(BARO_Measurements& baro_meas) override;
};