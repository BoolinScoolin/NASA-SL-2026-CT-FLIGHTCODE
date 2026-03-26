#include "Actuator.h"

CTServo::CTServo(uint8_t id)
: _id(id), _position_deg(0.0f) {
}

bool CTServo::initialize() {
    #ifdef SERVO_DISABLED
        return false;
    #endif
    SERVOSerial.begin(1000000, SERIAL_8N1);
    _st.pSerial = &SERVOSerial;
    return true;
}

void CTServo::writePosition(float deg) {
    #ifdef SERVO_DISABLED
        return ;
    #endif

    float ticks = deg*deg2servo + SERVO_HOME;
    _st.WritePosEx(_id, ticks, 4095, 255);
}

void CTServo::readPosition() {
    #ifdef SERVO_DISABLED
        return ;
    #endif

    int pos_ach = _st.ReadPos(_id);
    if (pos_ach < 0) {
        return ;  // read failed
    }
    _position_deg = (pos_ach - SERVO_HOME)/deg2servo;
}

float CTServo::getPositionDeg() const {
    return _position_deg;
}

