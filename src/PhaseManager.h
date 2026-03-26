#pragma once

/*
 * ============================================================================
 * PHASE MANAGER - Header File
 * ============================================================================
 * Handles flight phase detection and state machine transitions
 */

#ifndef PHASE_MANAGER_H
#define PHASE_MANAGER_H

#define ENABLE_PHASE_BUFFER  // or dont

#include <Arduino.h>
#include "KalmanFilter.h"
#include "main.h"
#include "Navigation.h"

extern const char* phaseNames[];

// ============================================================================
// PHASE DETECTION CONFIGURATION
// ============================================================================

// #define CONTROL_TEST_FLAG

#define PHASE_BUFFER_SIZE 50        // 0.5 seconds at 100 Hz

// Liftoff detection (ARMED → POWERED_ASCENT)
#define LIFTOFF_ACCEL_THRESHOLD 20.0f      // m/s² - adjust from motor specs
#define LIFTOFF_ALT_THRESHOLD 3.0f        // meters AGL

// Burnout detection (POWERED_ASCENT → COASTING)
#define BURNOUT_TIME_MIN 5000000          // 3 seconds (microseconds) for testing

// Control test phase length
#define CONTROL_TEST_TIME_US 3000000

// Lockouts
#define PITCH_RATE_RPS_LOCKOUT_THRESHOLD 3.0f
#define PITCH_ANGLE_DEG_LOCKOUT_THRESHOLD 45.0f // this may become a lookup table
const float CTHETA_THRESHOLD = cosf(PITCH_ANGLE_DEG_LOCKOUT_THRESHOLD*PI/180.0f);

// Apogee detection (COASTING → DESCENT)
#define APOGEE_SAMPLES_CHECK 10           // Check last 10 samples
#define APOGEE_ALT_DECREASE_THRESHOLD 0.2f // Altitude must decrease by at least 0.3m
#define APOGEE_VELOCITY_THRESHOLD -1.0f   // Velocity must be consistently negative

// Landing detection (DESCENT → LANDED)
#define LANDING_ALT_THRESHOLD 10.0f        // meters AGL (generous threshold)
#define LANDING_SAMPLES_CHECK 50          // Check last 20 samples (0.2 seconds at 100 Hz)
#define LANDING_ALT_CHANGE_MAX 0.03f        // Altitude change < 0.5m over samples
#define LANDING_VEL_CHANGE_MAX 0.1f        // Velocity change < 0.5m/s over samples

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Phase detection data (for analysis only)
struct PhaseDetectionData {
    float altitude;
    float velocity;
    float accelMagnitude;
    unsigned long timestamp;
};

// ============================================================================
// EXTERNAL VARIABLES (declared in PhaseManager.cpp)
// ============================================================================

extern unsigned long phaseStartTime;
extern unsigned long liftoffTime;
extern float maxAltitude;
extern unsigned long lastMovementTime;

#ifdef ENABLE_PHASE_BUFFER
extern PhaseDetectionData phaseBuffer[PHASE_BUFFER_SIZE];
extern int phaseBufferIndex;
extern bool phaseBufferFilled;
#endif

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Initialization
void initializePhaseManager();

// Main phase update (call at 100 Hz from control loop)
void updateFlightPhase(INS_State& ins, BARO_Measurements& baro_meas);

// Control check
bool shouldControl();

bool checkLockout(INS_State& ins, FlightPhase& currentPhase);

// Phase buffer functions (if enabled)
#ifdef ENABLE_PHASE_BUFFER
void addToPhaseBuffer(float altitude, float velocity, float accelMag);
PhaseDetectionData getPhaseDataFromPast(int samplesAgo);
bool isAltitudeDecreasingFor(int numSamples);
bool isVelocityNegativeFor(int numSamples);
float getAltitudeChange(int numSamples);
float getAverageAcceleration(int numSamples);
#endif

#endif // PHASE_MANAGER_H
