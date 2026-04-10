#include "logging.h"

char log_filename[32] = {0};
File output_file;

void makeOutputFile() {
    // Setup SD Card
    if (!SD.begin(BUILTIN_SDCARD)) {
        tone(BUZZER_PIN, NOTE_E8, 200);
        delay(200);
        tone(BUZZER_PIN, NOTE_C8, 200);
        delay(200);
        tone(BUZZER_PIN, NOTE_A7, 500);
        delay(2000);

        // Two beeps for bad SD Card read
        tone(BUZZER_PIN, NOTE_D8, 500);
        delay(600);
        tone(BUZZER_PIN, NOTE_D8, 500);
        while (true);
    }

    generateFilename();
    output_file = SD.open(log_filename, FILE_WRITE);
}

void generateFilename() {
    uint32_t idx = 0;

    while (true) {
        snprintf(log_filename, sizeof(log_filename),
                 "Flight_Data_%lu.csv", idx);
        if (!SD.exists(log_filename)) {
            break;
        }
        idx++;
    }
}

void writeHeaders() {
    output_file.print("time_s");
    output_file.print(",");
    output_file.print("ax_b_mps2");
    output_file.print(",");
    output_file.print("ay_b_mps2");
    output_file.print(",");
    output_file.print("az_b_mps2");
    output_file.print(",");
    output_file.print("gx_b_rps");
    output_file.print(",");
    output_file.print("gy_b_rps");
    output_file.print(",");
    output_file.print("gz_b_rps");
    output_file.print(",");
    output_file.print("baro_alt_m");
    output_file.print(",");
    output_file.print("p3_n_m");
    output_file.print(",");
    output_file.print("v3_n_mps");
    output_file.print(",");
    output_file.print("a3_n_mps2");
    output_file.print(",");
    output_file.print("q0");
    output_file.print(",");
    output_file.print("q1");
    output_file.print(",");
    output_file.print("q2");
    output_file.print(",");
    output_file.print("q3");
    output_file.print(",");
    output_file.print("state");
    output_file.print(",");
    output_file.print("pred apogee");
    output_file.print(",");
    output_file.print("encoder");
    output_file.print(",");
    output_file.print("cmd pos");
    output_file.print(",");
    output_file.println("cmd rate");
    
    output_file.flush();
}
