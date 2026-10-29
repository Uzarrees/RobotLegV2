#ifndef ROBOT_SERVO_H
#define ROBOT_SERVO_H

#include <SCServo.h>

class CustomServo {
    public:
        // constructor with id, home position, safe range of movement in angles (0 - 360)
        // and the raw step position that corresponds to 0 degrees (offset)
        // dir is +1 or -1 if the servo turns opposite your angle convention
        CustomServo(uint8_t id, int minAngle,  int maxAngle, int home, 
                    int zeroOffset, int dir = 1);

        void attach(SMS_STS* bus);  /// attach the shared servo bus (call once in setup)

        void enable();                  // torque on
        void disable();                 // torque off

        void moveTo(float angle);         // move to an absolute angle (clamped)         // convenience: got o centre
        void moveFraction(float f);     // 0.0 = minAngle, 1.0 = maxAngle  
        void home();              // moves to the home position

        float readAngle();                // current position of the servo
        bool inRange(float angle);        // retrns true if position is inside the position limits
        
        uint8_t id() const {return _id;}

    private:
        // th only two functions that know about steps, offset and direction
        int angleToSteps(float angle);  // converts an angle given into steps for the servo
        float stepsToAngle(int steps);  // converts the steps given into a readable angle
        float clampAngle(float angle);    // checks for the angle to be valid

        SMS_STS* _bus = nullptr;
        uint8_t _id;
        float _home, _maxAngle, _minAngle;
        int _zeroOffset, _dir;    

        static constexpr float STEPS_PER_DEGREE = 4096.0f / 360.0f;     
        const int SPEED = 2400;
        const int ACC = 50;
};

#endif