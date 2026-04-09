/*
 * ============================================================================
 * PHASE MANAGER - Implementation File
 * ============================================================================
 * Handles flight phase detection and state machine transitions
 */

#include "PhaseManager.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

const char* phaseNames[] = {
    "GROUND_IDLE",
    "ARMED",
    "POWERED_ASCENT",
    "COASTING",
    "DESCENT",
    "LANDED"
};

//FlightPhase currentPhase = GROUND_IDLE;
unsigned long phaseStartTime = 0;
unsigned long liftoffTime = 0;
float maxAltitude = 0.0f;
unsigned long lastMovementTime = 0;

#ifdef ENABLE_PHASE_BUFFER
PhaseDetectionData phaseBuffer[PHASE_BUFFER_SIZE];
int phaseBufferIndex = 0;
bool phaseBufferFilled = false;
#endif

// ============================================================================
// INITIALIZATION
// ============================================================================

void initializePhaseManager() {
    currentPhase = GROUND_IDLE;
    phaseStartTime = 0;
    liftoffTime = 0;
    maxAltitude = 0.0f;
    lastMovementTime = 0;
    
    #ifdef ENABLE_PHASE_BUFFER
    phaseBufferIndex = 0;
    phaseBufferFilled = false;
    #endif
}

// ============================================================================
// PHASE DETECTION BUFFER FUNCTIONS
// ============================================================================

#ifdef ENABLE_PHASE_BUFFER

void addToPhaseBuffer(float altitude, float velocity, float accelMag) {
    phaseBuffer[phaseBufferIndex].altitude = altitude;
    phaseBuffer[phaseBufferIndex].velocity = velocity;
    phaseBuffer[phaseBufferIndex].accelMagnitude = accelMag;
    phaseBuffer[phaseBufferIndex].timestamp = micros();
    
    phaseBufferIndex = (phaseBufferIndex + 1) % PHASE_BUFFER_SIZE;
    
    if (phaseBufferIndex == 0) {
        phaseBufferFilled = true;
    }
}

PhaseDetectionData getPhaseDataFromPast(int samplesAgo) {
    if (samplesAgo < 0 || samplesAgo >= PHASE_BUFFER_SIZE) {
        return phaseBuffer[(phaseBufferIndex - 1 + PHASE_BUFFER_SIZE) % PHASE_BUFFER_SIZE];
    }
    
    int index = (phaseBufferIndex - 1 - samplesAgo + PHASE_BUFFER_SIZE) % PHASE_BUFFER_SIZE;
    return phaseBuffer[index];
}

bool isAltitudeDecreasingFor(int numSamples) {
    if (!phaseBufferFilled && phaseBufferIndex < numSamples) {
        return false;
    }
    
    for (int i = 1; i < numSamples; i++) {
        PhaseDetectionData current = getPhaseDataFromPast(i - 1);
        PhaseDetectionData previous = getPhaseDataFromPast(i);
        
        if (current.altitude >= previous.altitude) {
            return false;
        }
    }
    
    return true;
}

bool isVelocityNegativeFor(int numSamples) {
    if (!phaseBufferFilled && phaseBufferIndex < numSamples) {
        return false;
    }
    
    for (int i = 0; i < numSamples; i++) {
        PhaseDetectionData data = getPhaseDataFromPast(i);
        if (data.velocity >= 0.0f) {
            return false;
        }
    }
    
    return true;
}

float getAltitudeChange(int numSamples) {
    if (!phaseBufferFilled && phaseBufferIndex < numSamples) {
        return 0.0f;
    }
    
    PhaseDetectionData current = getPhaseDataFromPast(0);
    PhaseDetectionData past = getPhaseDataFromPast(numSamples - 1);
    
    return current.altitude - past.altitude;
}

float getVelocityChange(int numSamples) {
    if (!phaseBufferFilled && phaseBufferIndex < numSamples) {
        return 0.0f;
    }
    
    PhaseDetectionData current = getPhaseDataFromPast(0);
    PhaseDetectionData past = getPhaseDataFromPast(numSamples - 1);
    
    return abs(current.velocity - past.velocity);
}

bool isStableFor(int numSamples, float maxAltChange, float maxVelChange) {
    if (!phaseBufferFilled && phaseBufferIndex < numSamples) {
        return false;  // Not enough data yet
    }
    
    // Get min and max altitude over the samples
    float minAlt = getPhaseDataFromPast(0).altitude;
    float maxAlt = minAlt;
    float minVel = getPhaseDataFromPast(0).velocity;
    float maxVel = minVel;
    
    for (int i = 1; i < numSamples; i++) {
        PhaseDetectionData data = getPhaseDataFromPast(i);
        
        if (data.altitude < minAlt) minAlt = data.altitude;
        if (data.altitude > maxAlt) maxAlt = data.altitude;
        if (data.velocity < minVel) minVel = data.velocity;
        if (data.velocity > maxVel) maxVel = data.velocity;
    }
    
    float altRange = maxAlt - minAlt;
    float velRange = maxVel - minVel;
    
    // Both altitude and velocity must be stable
    return (altRange < maxAltChange) && (velRange < maxVelChange);
}

float getAverageAcceleration(int numSamples) {
    if (!phaseBufferFilled && phaseBufferIndex < numSamples) {
        numSamples = phaseBufferIndex;
    }
    
    if (numSamples == 0) return 0.0f;
    
    float sum = 0.0f;
    for (int i = 0; i < numSamples; i++) {
        sum += getPhaseDataFromPast(i).accelMagnitude;
    }
    
    return sum / numSamples;
}

#endif  // ENABLE_PHASE_BUFFER

// ============================================================================
// FLIGHT PHASE MANAGER
// ============================================================================

void updateFlightPhase(INS_State& ins, BARO_Measurements& baro_meas) {
    unsigned long now = micros();
    float accelMagnitude = ins.accel_2_norm;
    
    #ifdef ENABLE_PHASE_BUFFER
    addToPhaseBuffer(-ins.p3_n_m, -ins.v3_n_mps, accelMagnitude);
    #endif
    
    // Track max altitude
    if (-ins.p3_n_m > maxAltitude) {
        maxAltitude = -ins.p3_n_m;
    }
    
    // ========================================================================
    // STATE MACHINE
    // ========================================================================
    
    switch(currentPhase) {
        case GROUND_IDLE:
            // Should never be here (auto-armed in setup)
            break;
            
        case ARMED: {
            // ================================================================
            // LIFTOFF DETECTION: ARMED → POWERED_ASCENT
            // Require BOTH high acceleration AND altitude gain
            // ================================================================
            bool highAcceleration = (accelMagnitude > LIFTOFF_ACCEL_THRESHOLD);
            // bool altitudeGain = (ins.p3_n_m > LIFTOFF_ALT_THRESHOLD);
            bool altitudeGain = true;

            if (highAcceleration && altitudeGain) {
                currentPhase = POWERED_ASCENT;
                liftoffTime = now;
                phaseStartTime = now;
                digitalWrite(RGB_R_PIN, HIGH);
                tone(BUZZER_PIN, NOTE_G8, 3000);
            }
        }
            break;
            
        case POWERED_ASCENT: {
            // ================================================================
            // BURNOUT DETECTION: POWERED_ASCENT -> COASTING
            // Motor burn time
            // ================================================================
            bool minBurnTime = ((now - liftoffTime) > BURNOUT_TIME_MIN);

            if (minBurnTime) {
                if (fabs(-ins.p3_n_m) < 10 && fabs(-ins.v3_n_mps) < 10) {
                    currentPhase = ARMED;
                    phaseStartTime = now;
                }
                else {
                    #ifdef CONTROL_TEST_FLAG
                    currentPhase = CONTROL_TEST;
                    #else
                    currentPhase = COASTING;
                    #endif
                    digitalWrite(RGB_R_PIN, LOW);
                    digitalWrite(RGB_G_PIN, HIGH);
                    phaseStartTime = now;
                }
            }
        }
            break;

        case CONTROL_TEST: {
            // send max actuation signal
            controller.commandFlapAngle(MAX_FLAP_ANGLE);
            servo.writePosition(controller.getActuatorCommand());

            // switch phase after timer finishes
            if (now - phaseStartTime > CONTROL_TEST_TIME_US) {
                currentPhase = COASTING;
                phaseStartTime = now;
            }
        }
            break;
            
        case COASTING: {
            #ifdef ENABLE_PHASE_BUFFER
            bool velocityNegative = isVelocityNegativeFor(APOGEE_SAMPLES_CHECK);
            bool altitudeDecreasing = isAltitudeDecreasingFor(APOGEE_SAMPLES_CHECK);
            float altChange = getAltitudeChange(APOGEE_SAMPLES_CHECK);
            bool significantDrop = (altChange < -APOGEE_ALT_DECREASE_THRESHOLD);
            
            // All three conditions must be met for robust apogee detection
            if (velocityNegative && altitudeDecreasing && significantDrop) {
                controller.commandFlapAngle(MIN_FLAP_ANGLE);
                servo.writePosition(controller.getActuatorCommand());
                currentPhase = DESCENT;
                phaseStartTime = now;
            }

            #else
            // Fallback: simple velocity check (less robust)
            bool descendingVelocity = (-ins.v3_n_mps < APOGEE_VELOCITY_THRESHOLD);
            
            if (descendingVelocity) {
                currentPhase = DESCENT;
                phaseStartTime = now;
            }
            #endif
        }
            break;
            
        case DESCENT: {
            // ================================================================
            // LANDING DETECTION: DESCENT → LANDED
            // altitude and velocity must be STABLE
            // Check if values barely change over last N samples
            // ================================================================
            
            bool lowAltitude = (-ins.p3_n_m < LANDING_ALT_THRESHOLD);
            
            #ifdef ENABLE_PHASE_BUFFER
            bool isStable = isStableFor(LANDING_SAMPLES_CHECK, 
                                       LANDING_ALT_CHANGE_MAX, 
                                       LANDING_VEL_CHANGE_MAX);
            
            if (lowAltitude && isStable) {
                currentPhase = LANDED;
                phaseStartTime = now;
                tone(BUZZER_PIN, NOTE_G8, 10000);
                delay(10000);
                output_file.flush();
                output_file.close();
                delay(2000);
                tone(BUZZER_PIN, NOTE_E8, 200);
                delay(200);
                tone(BUZZER_PIN, NOTE_C8, 200);
                delay(200);
                tone(BUZZER_PIN, NOTE_A7, 500);
                delay(2000);
            }
            #else
            // Fallback: simple threshold check (less robust)
            bool lowVelocity = (abs(ins.v3_n_mps) < 3.0f);
            
            if (lowAltitude && lowVelocity) {
                if (lastMovementTime == 0) {
                    lastMovementTime = now;
                } else if ((now - lastMovementTime) > 2000000) {
                    currentPhase = LANDED;
                    phaseStartTime = now;
                }
            } else {
                lastMovementTime = 0;
            }
            #endif
        }
            break;
            
        case LANDED: {
            // Stay in landed state
            break;
        
        }

        case ABORTED:
            //Retract airbrakes

            break;
    }
}

bool checkLockout(INS_State& ins, FlightPhase& currentPhase) {
    
    // // Condition 1: pitching too fast
    // float pitch_rate_rps = sqrt(ins.q_b_rps*ins.q_b_rps + ins.r_b_rps*ins.r_b_rps);
    // if (pitch_rate_rps > PITCH_RATE_RPS_LOCKOUT_THRESHOLD) {
    //     currentPhase = ABORTED;
    // }

    // // Condition 2: pitch angle too great
    // float ctheta = 2*(ins.q_nb.q0*ins.q_nb.q2 - ins.q_nb.q1*ins.q_nb.q3);

    // if (ctheta < CTHETA_THRESHOLD) {
    //     currentPhase = ABORTED;
    //     return true;
    // }

    return false;
}
