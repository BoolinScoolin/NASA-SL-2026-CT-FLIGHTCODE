#pragma once

#include <Arduino.h>
#include "Quaternion.h"
#include "Measurements.h"
#include "imu_calibration.h"
#include "pins.h"

static const float gz_n_mps2 = 9.80665;
static const Quaternion q_sb = {0.0f, 0.0f, 0.0f, 1.0f}; 
static const Quaternion q_ne = {0.0f, 0.7071068f, 0.7071068f, 0.0f}; 


struct INS_State {
    float p1_n_m, p2_n_m, p3_n_m;
    float v1_n_mps, v2_n_mps, v3_n_mps;
    Quaternion q_ns;
    Quaternion q_nb;
    float p_b_rps, q_b_rps, r_b_rps;
    float b1_g_rps, b2_g_rps, b3_g_rps;
    float b1_a_mps2, b2_a_mps2, b3_a_mps2;
    TUMBLE_Data tumble_calibration_data;
    float a1_n_mps2, a2_n_mps2, a3_n_mps2;
    float accel_2_norm;
};

struct Vector3 {
    float x, y, z;
};

// ============================================================================
// FLIGHT PHASES
// ============================================================================

enum FlightPhase {
    GROUND_IDLE,
    ARMED,
    POWERED_ASCENT,
    CONTROL_TEST,
    COASTING,        // Motor burned out, airbrakes active
    DESCENT,         // Past apogee, airbrakes retracted
    LANDED,
    ABORTED
};

void correctImuMeasurements(INS_State& ins, const IMU_Measurements& imu_meas);
void attitude_propagate(INS_State& ins, const IMU_Measurements& imu_meas);
void compute_attitude(INS_State& ins, FlightPhase& currentPhase, const IMU_Measurements& imu_meas);
void parse_reading(INS_State& ins, FlightPhase& currentPhase, const IMU_Measurements& imu_meas);
void initialize_orientation(INS_State& ins, IMU_Measurements& imu_meas);
void apply_imu_calibration(INS_State& ins);
void navigation_propagate(INS_State& ins, const IMU_Measurements& imu_meas);
void reset_INS(INS_State& ins, IMU_Measurements& imu_meas);

Vector3 cross_product(const Vector3& a, const Vector3& b);
void sensor2NED(Vector3& res, const Quaternion& q_eb, const Vector3& vec_raw_s);
void quat_rotate(Vector3& res, const Quaternion& q, const Vector3& v);
