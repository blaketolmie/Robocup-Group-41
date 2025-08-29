#include "myServo.h"
#include <Servo.h>
#include <Arduino.h>

// Servo object for gripper control
Servo gripperServo;

// Servo position tracking
static int currentPosition = 90;  // Start at middle position
static int targetPosition = 90;
static int servoVelocity = 10;    // Default velocity
static bool servoActivated = false;

// Position constants
#define SERVO_OUT_POSITION 0
#define SERVO_IN_POSITION 180
#define SERVO_PIN 9

void myservo_init() {
    gripperServo.attach(SERVO_PIN);
    gripperServo.write(currentPosition);
    servoActivated = false;
    Serial.println("Servo initialized on pin 9");
}

void myservo_callback() {
    // Update servo position gradually toward target
    if (currentPosition < targetPosition) {
        currentPosition = min(currentPosition + servoVelocity, targetPosition);
        gripperServo.write(currentPosition);
    } else if (currentPosition > targetPosition) {
        currentPosition = max(currentPosition - servoVelocity, targetPosition);
        gripperServo.write(currentPosition);
    }
    
    // Check if we've reached the target
    if (currentPosition == targetPosition) {
        servoActivated = false;
    }
}

void set_servo_velocity(int percentage) {
    // Convert percentage to actual velocity (1-20 degrees per update)
    servoVelocity = map(percentage, 0, 100, 1, 20);
    Serial.print("Servo velocity set to: ");
    Serial.println(servoVelocity);
}

void servo_out() {
    targetPosition = SERVO_OUT_POSITION;
    servoActivated = true;
    Serial.println("Servo moving OUT");
}

void servo_in() {
    targetPosition = SERVO_IN_POSITION;
    servoActivated = true;
    Serial.println("Servo moving IN");
}

bool is_servo_activated() {
    return servoActivated;
}

bool is_servo_in() {
    return (currentPosition >= SERVO_IN_POSITION - 5);
}

bool is_servo_out() {
    return (currentPosition <= SERVO_OUT_POSITION + 5);
}
