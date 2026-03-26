#pragma once

#include <String>
#include <stdint.h>
#include "Measurements.h"
#include "Sensors.h"
#include "SD.h"
//#include <assignment of drdy pin 

// tumble calibration from https://www.st.com/resource/en/design_tip/dm00253745-6point-tumble-sensor-calibration-stmicroelectronics.pdf

// stores the direction and magnitude of gravity in g's expected 
//float trueGravityVector[6][3]; 

// order of axis rotation for tumble calibration
enum axis {
        pos_X, 
        neg_X, 
        pos_Y, 
        neg_Y, 
        pos_Z, 
        neg_Z
};

// IMU data takes the form 

/*
        See the linked documentation to see how these arrays relate to the 3-axis IMU formula 
*/
struct TUMBLE_Data {
        float gainMatrix[3][3]; //stores the gains and cross gains calculated during the tumble calibration
        float invGainMatrix[3][3]; // stores the inverted gain matrix for computation
        float axisOffset[3]; // stores the offsets calculated along each axis during the tumble calibration
        float estimatedTrueAcceleration[3]; // stores the last estimate of true accleration
};

/*
        given some imu_data, update the estimated true accleration after computing it using matrix math 
*/
void estimate_true_accel(TUMBLE_Data* tumbl_d, const IMU_Measurements* imu_data);