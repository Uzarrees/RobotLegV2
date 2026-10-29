#include "customServo.h"

CustomServo::CustomServo(uint8_t id, int minAngle, int maxAngle, int home,
                         int zeroOffset, int dir)
    : _id(id), _minAngle(minAngle), _maxAngle(maxAngle), 
    _home(home), _zeroOffset(zeroOffset), _dir(dir) {}

void CustomServo::attach(SMS_STS* bus) {
    _bus = bus;
}

void CustomServo::enable() {
    if(_bus) _bus->EnableTorque(_id, 1);
}

void CustomServo::disable() {
    if(_bus) _bus->EnableTorque(_id, 0);
}

int CustomServo::angleToSteps(float angle) {
  return _zeroOffset + (int)(_dir * angle * STEPS_PER_DEGREE);
}

float CustomServo::stepsToAngle(int steps) {
  return (float)(steps - _zeroOffset) / (_dir * STEPS_PER_DEGREE);
}

float CustomServo::clampAngle(float angle) {
    if (angle < _minAngle) return _minAngle;
    if (angle > _maxAngle) return _maxAngle;
    return angle;
}

bool CustomServo::inRange(float angle) {
    return angle >= _minAngle && angle <= _maxAngle;
}

void CustomServo::moveTo(float angle) {
    if (!_bus) return;
    angle = clampAngle(angle);
    _bus->WritePosEx(_id, angleToSteps(angle), SPEED, ACC);
}

void CustomServo::home() {
    moveTo(_home);
}

void CustomServo::moveFraction(float f) {
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    float angle = _minAngle + (_maxAngle - _minAngle) * f;
    moveTo(angle);
}

float CustomServo::readAngle() {
    if (!_bus) return -1.0f;
    return stepsToAngle(_bus->ReadPos(_id));
}



