#include "imu_calibration.h"
#include "matrix_comp.h"
#include <Arduino.h>

#define TRUE_G_MPS2 9.80665

void estimate_true_accel(TUMBLE_Data* tumbl_d, const IMU_Measurements* imu_data)
{
        float unbiasedAcceleration[3];
        unbiasedAcceleration[0] = imu_data->accelX - tumbl_d->axisOffset[0];
        unbiasedAcceleration[1] = imu_data->accelY - tumbl_d->axisOffset[1];
        unbiasedAcceleration[2] = imu_data->accelZ - tumbl_d->axisOffset[2];

        // assume inverted gain matrix has already been calculated
        matrix_vector_multiplication_3x3(tumbl_d->estimatedTrueAcceleration, tumbl_d->invGainMatrix, unbiasedAcceleration);

        // // Prints
        // Serial.print(imu_data->accelX,6);
        // Serial.print(" ");
        // Serial.print(imu_data->accelY,6);
        // Serial.print(" ");
        // Serial.print(imu_data->accelZ,6);
        // Serial.print("   ");
        // Serial.print(unbiasedAcceleration[0],6);
        // Serial.print(" ");
        // Serial.print(unbiasedAcceleration[1],6);
        // Serial.print(" ");
        // Serial.print(unbiasedAcceleration[2],6);
        // Serial.print("   ");
        // Serial.print(tumbl_d->estimatedTrueAcceleration[0],6);
        // Serial.print(" ");
        // Serial.print(tumbl_d->estimatedTrueAcceleration[1],6);
        // Serial.print(" ");
        // Serial.println(tumbl_d->estimatedTrueAcceleration[2],6);
        
}

