#pragma once
#include "main.h"
#include "SCServo.h"
#include "pins.h"


// #define SERVO_HOME 1994  // update with closed position
// #define SERVO_MAX 2875 // update with max position (25 deg flap angle for barometer safety)

// const float deg2servo = 4096.0 / 360;
#define deg2servo (4096.0f/360)

class CTServo {
public:
    CTServo(uint8_t id);

    bool initialize();              // init servo object
    void setCloseTicks(uint16_t pos_ticks);   // set _close_ticks to current servo position
    void setOpenTicks(uint16_t pos_ticks);    // set _open_ticks to current servo position
    void writePosition(float deg);  // write servo deg
    void readPosition();     // servo deg
    void slamShut();        // commands servo to 0 ticks at full speed and accel
    void slamOpen();        // command servo to 4095 (full) ticks at full speed and accel
    uint16_t getCurrentTicks();  // returns servo position in ticks
    uint16_t getCurrentCurrent();  // returns servo current in ??
    
    // getters
    float getPositionDeg() const;
    uint16_t getCloseTicks() const;
    uint16_t getOpenTicks() const;


private:

    uint16_t _close_ticks;
    uint16_t _open_ticks;

    uint8_t _id;                    // scservo id
    float   _position_deg;          
    SMS_STS _st;                    // servo object
};


