#pragma once

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Arduino.h>
#include <stdint.h>
#include <matrix_comp.h>

class KalmanFilter {
public:
    KalmanFilter();

    // Safety-hardened update function
    void update(float z_measured);
    void predict(uint32_t current_micros);

    float getAltitudeError() { return x[0]; }
    float getVelocityError() { return x[1]; }
    float getAccelBias() {return x[2]; }

    // Call this in setup() to set the starting ground altitude
    void initialize(float initial_alt);

private:
    // State: x = [altitude, velocity]
    float x[3];

    // Covariance
    float P[3][3];

    // Process noise covariance
    float Q[3][3];

    // Measurement noise (scalar)
    float R;

    uint32_t last_micros;  // Can maybe comment this out in the new implementation
    uint32_t last_predict_micros;
    bool is_initialized;

    // Helpers
    static inline void mat2_zero(float M[2][2]) {
        M[0][0] = 0.0f; M[0][1] = 0.0f;
        M[1][0] = 0.0f; M[1][1] = 0.0f;
    }

    static inline void mat2_identity(float M[2][2]) {
        M[0][0] = 1.0f; M[0][1] = 0.0f;
        M[1][0] = 0.0f; M[1][1] = 1.0f;
    }
    
    static inline void mat3_zero(float M[3][3]) {
        M[0][0] = 0.0f; M[0][1] = 0.0f; M[0][2] = 0.0f;
        M[1][0] = 0.0f; M[1][1] = 0.0f; M[1][2] = 0.0f;
        M[2][0] = 0.0f; M[2][1] = 0.0f; M[2][2] = 0.0f;
    }

    static inline void mat3_identity(float M[3][3]) {
        M[0][0] = 1.0f; M[0][1] = 0.0f; M[0][2] = 0.0f;
        M[1][0] = 0.0f; M[1][1] = 1.0f; M[1][2] = 0.0f;
        M[2][0] = 0.0f; M[2][1] = 0.0f; M[2][2] = 1.0f;
    }
};

#endif
