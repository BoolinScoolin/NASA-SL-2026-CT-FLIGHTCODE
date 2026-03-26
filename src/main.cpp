#include <Arduino.h>
#include "main.h"

IMU_Measurements imu_meas = {0};
BARO_Measurements baro_meas = {0};
INS_State ins = {0};
FlightPhase currentPhase = ARMED;
ApogeeController controller;

IntervalTimer read_baro;

uint32_t now = 0;
uint32_t time_setup_complete_us = 0;

// Calibration data
TUMBLE_Data calib_data = {0};
KalmanFilter KF;

volatile bool read_baro_flag = false;
bool read_imu_flag = false;
volatile bool update_phase_flag = false;
volatile bool write_data_flag = false;

// Servo
CTServo servo(SERVO_ID);

//pick one in main.h
#ifdef REAL_SENSORS_FLAG
RealSensors backend;
#endif
#ifdef SIL_SENSORS_FLAG
SIL_Sensors backend;
IntervalTimer read_imu_SIL;
#endif
#if defined(REAL_SENSORS_FLAG) && defined(SIL_SENSORS_FLAG)
#error "REAL_SENSORS_FLAG and SIL_SENSORS_FLAG cannot both be defined. Check main.h"
#endif

void new_imu_reading() {
    now = micros() - time_setup_complete_us;
    readIMU(imu_meas);  // updates imu_meas with latest readings
    parse_reading(ins, currentPhase, imu_meas);  // computes attitude
    KF.predict(now);

    if (read_baro_flag) {
        readBarometer(baro_meas);  // updates baro_meas with latest barometer readings
        float z_alt_m = baro_meas.baroAltitude - groundAltitude;  // subtracts tare
        KF.update(z_alt_m);
        float dz_m = KF.getAltitudeError();
        float dvz_mps = KF.getVelocityError();
        float dba_mps2 = KF.getAccelBias();
        ins.p3_n_m += dz_m;
        ins.v3_n_mps += dvz_mps;
        ins.b3_a_mps2 += dba_mps2;
        read_baro_flag = false; 
    }

    update_phase_flag = true;

    // Logging
    static uint32_t k = 0;
    k++;
    if (k >= 10) {
        write_data_flag = true;
        k = 0;
    }   
}

void read_baro_ISR() {
    read_baro_flag = true;
}

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    attach_sensor_backend(&backend);

    // LED Initialize
    pinMode(RGB_R_PIN, OUTPUT);
    pinMode(RGB_G_PIN, OUTPUT);
    pinMode(RGB_B_PIN, OUTPUT);
    digitalWrite(RGB_R_PIN, HIGH);

    // Chime to signify start of initialization
    pinMode(BUZZER_PIN, OUTPUT);
    tone(BUZZER_PIN, NOTE_B7, 50);
    delay(50);
    tone(BUZZER_PIN, NOTE_D8, 50);
    delay(50);
    tone(BUZZER_PIN, NOTE_G8, 100);
    delay(2000);
    analogWriteFrequency(BUZZER_PIN, NOTE_G8);


    // Setup Servo
    Serial.begin(115200);
    Serial.println("Entering...");
    if(!servo.initialize()) {
        Serial.println("Servo disabled.");
        tone(BUZZER_PIN, NOTE_C8, 50);
        delay (100);
        tone(BUZZER_PIN, NOTE_C8, 500);
        delay (1000);
    }

    // // Open Close Loop (COMMENT OUT IN FLIGHT)
    // while (true) {
    //     tone(BUZZER_PIN, NOTE_A8, 50);

    //     servo.writePosition(0);
    //     delay(2000);
    //     servo.readPosition();
    //     Serial.println(servo.getPositionDeg());
    //     delay(2000);

    //     servo.writePosition(5.0);
    //     delay(2000);
    //     servo.readPosition();
    //     Serial.println(servo.getPositionDeg());
    //     delay(2000);

    //     servo.writePosition(10.0);
    //     delay(2000);
    //     servo.readPosition();
    //     Serial.println(servo.getPositionDeg());
    //     delay(2000);

    //     servo.writePosition(15.0);
    //     delay(2000);
    //     servo.readPosition();
    //     Serial.println(servo.getPositionDeg());
    //     delay(2000);

    //     servo.writePosition(20.0);
    //     delay(2000);
    //     servo.readPosition();
    //     Serial.println(servo.getPositionDeg());
    //     delay(2000);

    //     servo.writePosition(25.0);
    //     delay(2000);
    //     servo.readPosition();
    //     Serial.println(servo.getPositionDeg());
    //     delay(2000);
    // }

    // setup controller
    controller.initialize();

    if (!initializeIMU()) {
        digitalWrite(RGB_R_PIN, LOW);
        while (1) {
            digitalWrite(RGB_B_PIN, !digitalRead(RGB_B_PIN)); // blink LED if sensor init fails
            tone(BUZZER_PIN, NOTE_B7, 50);      // Every 2 seconds for bad sensor
            delay(2000);
        }
    }

    makeOutputFile();
    writeHeaders();

    // Attach barometer intervaltimer
    #ifdef BARO_DISABLED
        Serial.print("Barometer disabled.");
        tone(BUZZER_PIN, NOTE_A7, 50);
        delay (100);
        tone(BUZZER_PIN, NOTE_A7, 500);
        delay (1000);
    #else
        if (!initializeBarometer()) {
            digitalWrite(RGB_R_PIN, LOW);
            while (1) {
                digitalWrite(RGB_B_PIN, !digitalRead(RGB_B_PIN)); // blink LED if sensor init fails
                tone(BUZZER_PIN, NOTE_B7, 50);      // Every 2 seconds for bad sensor
                delay(4000);
            }
        }
        performBarometerTare();
        read_baro.begin(read_baro_ISR, 100000);
    #endif

    apply_imu_calibration(ins);
    performIMUTare();
    initialize_orientation(ins, imu_meas);

    // Attach IMU Interrupt
    #ifdef REAL_SENSORS_FLAG
    if (digitalRead(IMU_DRDY_PIN)) readIMU(imu_meas);
    attachInterrupt(IMU_DRDY_PIN, new_imu_reading, RISING);
    #endif

    // Attach SIL IMU intervaltimer
    #ifdef SIL_SENSORS_FLAG
    read_imu_SIL.begin(new_imu_reading,10000);
    #endif


    tone(BUZZER_PIN, NOTE_A8, 100);
    delay(50);
    tone(BUZZER_PIN, NOTE_G8, 100);
    delay(1000);
    time_setup_complete_us = micros();
}

void loop() {

    now = micros() - time_setup_complete_us;

    static uint32_t last_print_us = 0;

    // State detection
    if (update_phase_flag) {
        updateFlightPhase(ins, baro_meas);
        update_phase_flag = false;
    }

    if (currentPhase == ARMED) {

        // this is designed to re-tare ground altitude every minute
        static RollingMeanFifo<600> groundAlt;
        static uint32_t last_groundalt_update_us = micros();
        groundAlt.push(baro_meas.baroAltitude);
        if (now - last_groundalt_update_us > 60*1000000) {
            groundAltitude = groundAlt.mean();
            last_groundalt_update_us = micros();
        }

        // This just blinks the LED if its armed
        static uint32_t last_blink_update_us = micros();
        if (now - last_blink_update_us > 1*1000000) {
            digitalWrite(RGB_R_PIN, !digitalRead(RGB_R_PIN));
            last_blink_update_us = micros();
        }

        // This just beeps if its armed
        static uint32_t last_beep_update_us = micros();
        static bool beep_flag = false;
        if (now - last_beep_update_us > 5*1000000) {
            //tone(BUZZER_PIN, NOTE_G8, 100);
            analogWrite(BUZZER_PIN, 128);
            last_beep_update_us = micros();
            beep_flag = true;
        }
        if (beep_flag && now - last_beep_update_us > 200000) {
            analogWrite(BUZZER_PIN, 0);
            beep_flag = false;
        }
    }

    // Controls stuff
    if (controller.shouldControl(currentPhase)) {
        if(checkLockout(ins, currentPhase)) {
            controller.shouldControl(currentPhase);
        }
        else if (currentPhase == COASTING) { 
            controller.update(ins, currentPhase);
            servo.writePosition(controller.getActuatorCommand());
        }
    }

    // Write data
    static uint32_t c = 0;  // Flush counter
    if (false && write_data_flag) {
        servo.readPosition();
        output_file.print(now*1.0e-6f,6);
        output_file.print(",");
        output_file.print(imu_meas.accelX, 6);
        output_file.print(",");
        output_file.print(imu_meas.accelY, 6);
        output_file.print(",");
        output_file.print(imu_meas.accelZ, 6);
        output_file.print(",");
        output_file.print(imu_meas.gyroX, 6);
        output_file.print(",");
        output_file.print(imu_meas.gyroY, 6);
        output_file.print(",");
        output_file.print(imu_meas.gyroZ, 6);
        output_file.print(",");
        output_file.print(baro_meas.baroAltitude, 6);
        output_file.print(",");
        output_file.print(ins.p3_n_m);
        output_file.print(",");
        output_file.print(ins.v3_n_mps);
        output_file.print(",");
        output_file.print(ins.q_nb.q0, 4);
        output_file.print(",");
        output_file.print(ins.q_nb.q1, 4);
        output_file.print(",");
        output_file.print(ins.q_nb.q2, 4);
        output_file.print(",");
        output_file.print(ins.q_nb.q3, 4);
        output_file.print(",");
        output_file.print(currentPhase);
        output_file.print(",");
        output_file.print(servo.getPositionDeg());
        output_file.print(",");
        output_file.println(controller.getActuatorCommand());

        c++;
        if (c >= 200) {
            output_file.flush();
            c = 0;
        }

        write_data_flag = false;
    }

    // DEBUG PRINTS
    if (false && now - last_print_us > 90000) {

        float roll_rad;
        float pitch_rad;
        float yaw_rad;
        quat2eul(ins.q_nb, roll_rad, pitch_rad, yaw_rad);

        // Serial.print(roll_rad*180/PI, 4);
        // Serial.print(" ");
        // Serial.print(pitch_rad*180/PI,4);
        // Serial.print(" ");
        // Serial.print(yaw_rad*180/PI,4);
        // Serial.print("    ");

        // Serial.print(ins.q_nb.q0, 4);
        // Serial.print(" ");
        // Serial.print(ins.q_nb.q1, 4);
        // Serial.print(" ");
        // Serial.print(ins.q_nb.q2, 4);
        // Serial.print(" ");
        // Serial.print(ins.q_nb.q3, 4);
        // Serial.print("     ");

        // float pitch_rate_rps = sqrt(ins.q_b_rps*ins.q_b_rps + ins.r_b_rps*ins.r_b_rps);
        // Serial.print(pitch_rate_rps, 4);
        // Serial.print("   ");

        // Serial.print(ins.accel_2_norm);
        // Serial.print("   ");

        // Serial.print(sil_true_alt());
        // Serial.print("  ");

        Serial.print(KF.getAltitudeError());
        Serial.print("    ");
        // Serial.print(baro_meas.baroAltitude - groundAltitude, 4);
        // Serial.print("  ");
        // Serial.print(ins.p1_n_m, 2);
        // Serial.print("  ");
        // Serial.print(ins.p2_n_m, 2);
        // Serial.print("  ");
        // Serial.print(ins.p3_n_m, 2);
        // Serial.print("        ");
        // Serial.print(ins.v3_n_mps, 4);
        // Serial.print("        ");
        // Serial.print(ins.a1_n_mps2, 4);
        // Serial.print(" ");
        // Serial.print(ins.a2_n_mps2, 4);
        // Serial.print(" ");
        // Serial.print(ins.a3_n_mps2, 4);
        // Serial.print("                 ");
        // Serial.print(ins.b3_a_mps2);
        // Serial.print("      ");

        // Serial.print(imu_meas.IMU_dt_us);
        // Serial.print("   ");

        // Serial.print(servo.getPositionDeg());
        // Serial.print("   ");

        // Serial.print(ins.tumble_calibration_data.estimatedTrueAcceleration[0]);
        // Serial.print(" ");
        // Serial.print(ins.tumble_calibration_data.estimatedTrueAcceleration[1]);
        // Serial.print(" ");
        // Serial.print(ins.tumble_calibration_data.estimatedTrueAcceleration[2]);
        // Serial.print("     ");

        // Serial.print(imu_meas.accelX);
        // Serial.print(" ");
        // Serial.print(imu_meas.accelY);
        // Serial.print(" ");
        // Serial.print(imu_meas.accelZ);
        // Serial.print("     ");


        // Serial.print(ins.p_b_rps,4);
        // Serial.print(" ");
        // Serial.print(ins.q_b_rps,4);
        // Serial.print(" ");
        // Serial.print(ins.r_b_rps,4);
        // Serial.print("    ");


        Serial.print(now/1.0e6f,3);
        Serial.print("    ");
        
        Serial.println(currentPhase);

        last_print_us = now;
    }

    // GUI PRINTS
    if (false && now - last_print_us > 9000) {

        //t_s,x_m,y_m,z_m,q0,q1,q2,q3
        Serial.print(now/1.0e6f, 3);
        Serial.print(",");
        Serial.print(ins.p1_n_m, 4);
        Serial.print(",");
        Serial.print(ins.p2_n_m, 4);
        Serial.print(",");
        Serial.print(ins.p3_n_m, 4);
        Serial.print(",");
        Serial.print(ins.q_nb.q0, 4);
        Serial.print(",");
        Serial.print(ins.q_nb.q1, 4);
        Serial.print(",");
        Serial.print(ins.q_nb.q2, 4);
        Serial.print(",");
        Serial.println(ins.q_nb.q3, 4);

        last_print_us = now;
    }
}