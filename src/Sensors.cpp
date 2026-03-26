#include "Sensors.h"

SensorBackend* g_backend = nullptr;

void attach_sensor_backend(SensorBackend* backend) {
    g_backend = backend;
}

bool initializeIMU() {
    return (g_backend && g_backend->initializeIMU());
}

bool performIMUTare() {
    return (g_backend && g_backend->performIMUTare());
}

bool readIMU(IMU_Measurements& imu_meas) {
    return (g_backend && g_backend->readIMU(imu_meas));
}

bool initializeBarometer() {
    return (g_backend && g_backend->initializeBarometer());
}

bool performBarometerTare() {
    return (g_backend && g_backend->performBarometerTare());
}

bool readBarometer(BARO_Measurements& baro_meas) {
    return (g_backend && g_backend->readBarometer(baro_meas));
}

float sil_true_alt() {
    return (g_backend && g_backend->sil_true_alt());
}
