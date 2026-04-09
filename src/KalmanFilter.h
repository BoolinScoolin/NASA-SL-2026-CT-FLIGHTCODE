#pragma once


#include <Arduino.h>
#include <stdint.h>
#include <matrix_comp.h>
#include "Navigation.h"

class KalmanFilter {
public:
    KalmanFilter();

    void update(float z_measured, INS_State& ins);
    void predict(uint32_t current_micros);
    void reset();

    void printP();

    float db;

private:

    // Covariance
    float P[3][3];

    // Helpers
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