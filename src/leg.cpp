#include "leg.h"
#include "kinematics.h"

Leg::Leg(CustomServo& hip, CustomServo& knee, float L1, float L2)
    : _hip(hip), _knee(knee), _L1(L1), _L2(L2) {}

void Leg::enable()  { _hip.enable(); _knee.enable(); }
void Leg::disable() { _hip.disable(); _knee.disable(); }
void Leg::home()    { _hip.home(); _knee.home(); }


void Leg::moveToAngles(float hipDeg, float kneeDeg) {
    _hip.moveTo(hipDeg);
    _knee.moveTo(kneeDeg);
}

bool Leg::moveToPoint(float x, float y) {
    float hipDeg, kneeDeg;
    if (!solveLeg2D(x, y, _L1, _L2, hipDeg, kneeDeg)) {
        Serial.println("ERROR - Point unreachable");
        return false;
    }

    moveToAngles(hipDeg, kneeDeg);
    Serial.print("Hip angle: ");
    Serial.print(hipDeg);
    Serial.print("\tKnee angle: ");
    Serial.println(kneeDeg);
    return true;
}

