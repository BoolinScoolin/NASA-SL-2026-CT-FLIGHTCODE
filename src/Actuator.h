#pragma once
#include "main.h"
#include "SCServo.h"
#include "pins.h"


#define SERVO_HOME 1994  // update with closed position
#define SERVO_MAX 2875 // update with max position (25 deg flap angle for barometer safety)

// const float deg2servo = 4096.0 / 360;
#define deg2servo (4096.0f/360)

#define SERVO_OPEN SERVO_MAX/deg2servo
#define SERVO_CLOSE SERVO_HOME/deg2servo

class CTServo {
public:
    CTServo(uint8_t id);

    bool initialize();              // init servo object
    void writePosition(float deg);  // write servo deg
    void readPosition();     // servo deg
    float getPositionDeg() const;


private:
    uint8_t _id;                    // scservo id
    float   _position_deg;          
    SMS_STS _st;                    // servo object
};


