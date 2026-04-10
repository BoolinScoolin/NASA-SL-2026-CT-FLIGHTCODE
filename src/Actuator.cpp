#include "Actuator.h"

CTServo::CTServo(uint8_t id)
: _id(id), _position_deg(0.0f) {
}

bool CTServo::initialize() {

    _close_ticks = 0;
    _open_ticks = 0;

    #ifdef SERVO_DISABLED
        return false;
    #endif

    SERVOSerial.begin(1000000, SERIAL_8N1);
    _st.pSerial = &SERVOSerial;
    return true;
}

void CTServo::setCloseTicks(uint16_t pos_ticks) {
    #ifdef SERVO_DISABLED
        return ;
    #endif

    _close_ticks = pos_ticks;
}

void CTServo::setOpenTicks(uint16_t pos_ticks) {
    #ifdef SERVO_DISABLED
        return ;
    #endif

    _open_ticks = pos_ticks;
}

void CTServo::writePosition(float deg) {
    #ifdef SERVO_DISABLED
        return ;
    #endif

    float ticks = deg*deg2servo + _close_ticks;
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
    _position_deg = (pos_ach - _close_ticks)/deg2servo;
}

void CTServo::slamShut() {
    #ifdef SERVO_DISABLED
        return ;
    #endif

    _st.WritePosEx(_id, 0, 4095, 255);

}

void CTServo::slamOpen() {
    #ifdef SERVO_DISABLED
        return ;
    #endif

    _st.WritePosEx(_id, 4095, 4095, 255);
}

uint16_t CTServo::getCurrentTicks() {
    #ifdef SERVO_DISABLED
        return ;
    #endif

    return _st.ReadPos(_id);
}

uint16_t CTServo::getCurrentCurrent() {
    #ifdef SERVO_DISABLED
        return ;
    #endif

    return _st.ReadCurrent(_id);
}

float CTServo::getPositionDeg() const {
    return _position_deg;
}

uint16_t CTServo::getCloseTicks() const {
    return _close_ticks;
}

uint16_t CTServo::getOpenTicks() const {
    return _open_ticks;
}

