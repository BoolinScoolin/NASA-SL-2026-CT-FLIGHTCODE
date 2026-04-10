#pragma once

// Includes
#include <Arduino.h>
#include "Measurements.h"
#include "Navigation.h"
#include "Sensors.h"
#include "Quaternion.h"
#include "imu_calibration.h"
#include "KalmanFilter.h"
#include "PhaseManager.h"
#include "Actuator.h"
#include "ApogeeController.h"
#include "logging.h"
#include "pins.h"
#include "SensorBackend.h"

// PICK ONE
#include "RealSensors.h"
// #include "SIL_sensors.h"

// DISABLE PERIPHERALS
// #define SERVO_DISABLED  // Disables sending servo commands
// #define BARO_DISABLED   // Disables barometer initialization and ISR


#define OUTPUT_FILENAME "Flight_Data.txt" // include .txt

// Initialize beeper 
const int NOTE_A7 = 3520;
const int NOTE_B7 = 3951;
const int NOTE_C8 = 4186;
const int NOTE_D8 = 4698;
const int NOTE_E8 = 5274;
const int NOTE_F8 = 5588;
const int NOTE_G8 = 6272;
const int NOTE_A8 = 7040;
const int NOTE_B8 = 7902;

// Data logging
//extern char log_filename[32];
extern File output_file;

// Initialize Servo Parameters
class CTServo;
extern CTServo servo; // create servo object

extern IMU_Measurements imu_meas;
extern BARO_Measurements baro_meas;
extern INS_State ins;
extern FlightPhase currentPhase;

class ApogeeController;
extern ApogeeController controller;


// Calibration data
extern TUMBLE_Data calib_data;
extern KalmanFilter KF;


template <size_t N>
struct RollingMeanFifo {
    float buf[N] = {0};
    size_t head = 0;
    size_t count = 0;
    float sum = 0.0f;

    void push(float x) {
        if (count < N) {
            // buffer not full yet
            buf[head] = x;
            sum += x;
            head = (head + 1) % N;
            count++;
        } else {
            // overwrite oldest (which is at head)
            sum -= buf[head];
            buf[head] = x;
            sum += x;
            head = (head + 1) % N;
        }
    }

    float mean() const {
        return (count == 0) ? 0.0f : (sum / (float)count);
    }

    bool full() const { return count == N; }
};
