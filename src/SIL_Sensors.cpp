#pragma once
#include "SIL_sensors.h"


extern uint32_t now;
//fake IMU methods
int SIL_Sensors::findTimeIndexBinary(float t_runtime) {
    if (SIL_N <= 1) return 0;
    if (t_runtime <= sil_time_s[0]) return 0;
    if (t_runtime >= sil_time_s[SIL_N-1]) return SIL_N-1;

    int lo = 0;
    int hi = SIL_N - 1;

    // Invariant: t[lo] <= t_runtime < t[hi]
    while (hi - lo > 1) {
        int mid = lo + (hi - lo) / 2;
        if (sil_time_s[mid] <= t_runtime) lo = mid;
        else hi = mid;
    }
    return lo;
}

void SIL_Sensors::readMessages(float t_s) {

    int k = findTimeIndexBinary(t_s);

    euler_rad[0]  = sil_euler_1_rad[k];
    euler_rad[1]  = sil_euler_2_rad[k];
    euler_rad[2]  = sil_euler_3_rad[k];

    accel_mps2[0] = sil_accel_x_mps2[k];
    accel_mps2[1] = sil_accel_y_mps2[k];
    accel_mps2[2] = sil_accel_z_mps2[k];

    gyro_rps[0]   = sil_gyro_x_rps[k];
    gyro_rps[1]   = sil_gyro_y_rps[k];
    gyro_rps[2]   = sil_gyro_z_rps[k];
}


float* SIL_Sensors::getEulerAngles() { return euler_rad; }
float* SIL_Sensors::getAcceleration() { return accel_mps2; }
float* SIL_Sensors::getRateOfTurn() { return gyro_rps; }

float SIL_Sensors::readAltitude(float sea_level_pressure_pa, float t_s) {
    int k = findTimeIndexBinary(t_s);
    return sil_alt_z_m[k];
}

float SIL_Sensors::sil_true_alt() {
    t = (now/1.0e6f) + 0*sil_time_s[0];
    int k = findTimeIndexBinary(t);
    return sil_true_z_m[k];
}

//Sensor methods
bool SIL_Sensors::initializeIMU() {
    return true;
}
bool SIL_Sensors::performIMUTare(){
    return true;
}
bool SIL_Sensors::readIMU(IMU_Measurements& imu_meas){
    t = (now/1.0e6f) + 0*sil_time_s[0];
    readMessages(t);

    float *acc  = getAcceleration();
    float *gyro = getRateOfTurn();

    // PLACEHOLDER for now
    float *eulerAngles = getEulerAngles();

    if (true) {
        // imu_meas.accelX = acc[0] - imuTare[0];
        // imu_meas.accelY = acc[1] - imuTare[1];
        // imu_meas.accelZ = acc[2] - imuTare[2];
        imu_meas.accelX = acc[0];
        imu_meas.accelY = acc[1];
        imu_meas.accelZ = acc[2];
        imu_meas.gyroX  = gyro[0];
        imu_meas.gyroY  = gyro[1];
        imu_meas.gyroZ  = gyro[2];
        imu_meas.roll_deg = eulerAngles[0]*180.0f/PI;
        imu_meas.pitch_deg = eulerAngles[1]*180.0f/PI;
        imu_meas.yaw_deg = eulerAngles[2]*180.0f/PI;
    } else {
        imu_meas.accelX = acc[0];
        imu_meas.accelY = acc[1];
        imu_meas.accelZ = acc[2];
        imu_meas.gyroX  = gyro[0];
        imu_meas.gyroY  = gyro[1];
        imu_meas.gyroZ  = gyro[2];
        imu_meas.roll_deg = eulerAngles[0]*180.0f/PI;
        imu_meas.pitch_deg = eulerAngles[1]*180.0f/PI;
        imu_meas.yaw_deg = eulerAngles[2]*180.0f/PI;
    }

    // Timer updates
    // now = micros();
    imu_meas.IMU_dt_us = now - imu_meas.last_IMU_reading_time_us;
    imu_meas.last_IMU_reading_time_us = now;

    //Serial.println(t, 6);

    return true;
}

bool SIL_Sensors::initializeBarometer(){
    return true;
};
bool SIL_Sensors::performBarometerTare(){
    return true;
};
bool SIL_Sensors::readBarometer(BARO_Measurements& baro_meas){
    t = (now/1.0e6f) + 0*sil_time_s[0];

    baro_meas.baroAltitude    = readAltitude(1013.25f, t);
    baro_meas.baroPressure    = 0;
    baro_meas.baroTemperature = 0; //unused possibly

    return true;
};