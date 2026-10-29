// ======================================================================
// RUN_MODE 2: Test mode
// 
// ======================================================================

#include <Arduino.h>
#include <SCServo.h>
#include "customServo.h"
#include "leg.h"

SMS_STS bus;


#define S_RXD 16    
#define S_TXD 17

// define servos with id, minAngle, maxAngle, home, offset position and dir
CustomServo hipServo(1, 0, 180, 90, 0, 1);     // servo 1
CustomServo kneeServo(2, 0, 180, 90, 0, 1); // servo 2

// link the servos to the leg with the link lenghs
Leg leg(hipServo, kneeServo, 10.5f, 4.5f);

// ============== Helpers ===============


// ============== Setup ================
void setup_test() {
    Serial.begin(115200);
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    bus.pSerial = &Serial1;
    delay(1000);

    hipServo.attach(&bus);
    kneeServo.attach(&bus);

    leg.enable();
    Serial.println("Torque enabled");

    leg.home();
    Serial.println("Homed");
}

// ================ Loop ================
void loop_test() {
    float X, Y;

    Serial.println("\nReaching point A (0, 15)");
    X = 0.0f;
    Y = 15.0f;
    leg.moveToPoint(X, Y);
    delay(2000);

    Serial.println("\nReaching point B (4.5, 10.5)");
    X = 4.5f;
    Y = 10.5f;
    leg.moveToPoint(X, Y);
    delay(2000);

    Serial.println("\nReaching point C (10.5, 4.5)");
    X = 10.5f;
    Y = 4.5f;
    leg.moveToPoint(X, Y);
    delay(2000);

    Serial.println("\nReaching point D (15, 0)");
    X = 15.0f;
    Y = 0.0f;
    leg.moveToPoint(X, Y);
    delay(2000);


    /*
    Serial.println("Reaching point A");
    // move one way
    leg.moveToAngles(90.0f, -90.0f);
    delay(2000);
    
    // move the other way
    Serial.println("Reaching point B");
    leg.moveToAngles(180.0f, 0.0f);
    delay(2000);
    */
}