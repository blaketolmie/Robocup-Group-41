// #include "Arduino.h"
// #include <Servo.h>

// // Motor pins
// #define LEFT_MOTOR_PIN 0
// #define RIGHT_MOTOR_PIN 1

// // Motor control values
// #define MOTOR_STOP 1500
// #define MOTOR_FORWARD 1025
// #define MOTOR_REVERSE 1975

// // Create servo objects
// Servo leftMotor;
// Servo rightMotor;

// void setup() {
//   Serial.begin(9600);
//   Serial.println("=== Simple Motor Test ===");
  
//   // Attach motors
//   leftMotor.attach(LEFT_MOTOR_PIN);
//   rightMotor.attach(RIGHT_MOTOR_PIN);
  
//   Serial.println("Motors attached to pins 0 and 1");
  
//   // Start with motors stopped
//   leftMotor.writeMicroseconds(MOTOR_STOP);
//   rightMotor.writeMicroseconds(MOTOR_STOP);
  
//   Serial.println("Motors stopped - waiting 3 seconds...");
//   delay(3000);
// }

// void loop() {
//   // Test forward movement
//   Serial.println("FORWARD: Both motors at 1025");
//   leftMotor.writeMicroseconds(MOTOR_FORWARD);
//   rightMotor.writeMicroseconds(MOTOR_FORWARD);
//   delay(3000);
  
//   // Stop
//   Serial.println("STOP: Both motors at 1500");
//   leftMotor.writeMicroseconds(MOTOR_STOP);
//   rightMotor.writeMicroseconds(MOTOR_STOP);
//   delay(2000);
  
//   // Test individual motors
//   Serial.println("LEFT ONLY: Left motor forward");
//   leftMotor.writeMicroseconds(MOTOR_FORWARD);
//   rightMotor.writeMicroseconds(MOTOR_STOP);
//   delay(2000);
  
//   Serial.println("STOP: Both motors stopped");
//   leftMotor.writeMicroseconds(MOTOR_STOP);
//   rightMotor.writeMicroseconds(MOTOR_STOP);
//   delay(2000);
  
//   Serial.println("RIGHT ONLY: Right motor forward");
//   leftMotor.writeMicroseconds(MOTOR_STOP);
//   rightMotor.writeMicroseconds(MOTOR_FORWARD);
//   delay(2000);
  
//   Serial.println("STOP: Both motors stopped");
//   leftMotor.writeMicroseconds(MOTOR_STOP);
//   rightMotor.writeMicroseconds(MOTOR_STOP);
//   delay(2000);
  
//   Serial.println("=== Test cycle complete ===\n");
//   delay(3000);
// }
