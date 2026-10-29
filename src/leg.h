#ifndef LEG_H
#define LEG_H

#include "customServo.h"

class Leg {
  public:
    // Holds references to the two servos and the link lengths.
    Leg(CustomServo& hip, CustomServo& knee, float L1, float L2);

    void enable();
    void disable();
    void home();

    // Command joints directly in angle space.
    void moveToAngles(float hipDeg, float kneeDeg);

    // Command a target foot position in the leg's own frame.
    // Returns false (and does nothing) if the point is unreachable.
    bool moveToPoint(float x, float y);

  private:
    CustomServo& _hip;
    CustomServo& _knee;
    float _L1, _L2;
};

#endif