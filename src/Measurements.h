#pragma once

#include <cstdint>

// Current measurements (holds latest sensor readings)
struct IMU_Measurements {
    // IMU
    float accelX, accelY, accelZ;  // m/s2
    float gyroX, gyroY, gyroZ;     // rad/s
    float roll_deg, pitch_deg, yaw_deg;
    uint32_t last_IMU_reading_time_us;
    uint32_t IMU_dt_us;
};

struct BARO_Measurements {
    // Barometer
    float baroAltitude;            // meters
    float baroPressure;            // hPa
    float baroTemperature;         // deg C
};

struct GPS_Measurements {
    // GPS
    float gpsLat;
    float gpsLon;
    float gpsAltitude;
    float gpsSpeed;
    uint8_t gpsSatellites;
    bool gpsHasFix;
    float gpsXMeters;
    float gpsYMeters;
};

