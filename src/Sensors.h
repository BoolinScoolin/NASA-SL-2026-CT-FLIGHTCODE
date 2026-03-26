#pragma once

#include "SensorBackend.h"
#include "Measurements.h"

void attach_sensor_backend(SensorBackend* backend);

bool initializeIMU();

bool performIMUTare();

bool readIMU(IMU_Measurements& imu_meas);

bool initializeBarometer();

bool performBarometerTare();

bool readBarometer(BARO_Measurements& baro_meas);

float sil_true_alt();



