#include "Arduino.h"
#include "motors.h"

  // Create servo objects for motor control
  Servo leftMotor;
  Servo rightMotor;


void motors_init() {
  Serial.println("Initializing motors...");
  leftMotor.attach(LEFT_MOTOR_ADDRESS);
  rightMotor.attach(RIGHT_MOTOR_ADDRESS);

  leftMotor.writeMicroseconds(1500); // Direct call since 1500 is outside our caps start at stop
  rightMotor.writeMicroseconds(1500);
  Serial.println("Motors initialized and stopped");
}



// Check speed limits
void check_speed_limits(int speed) {
  if (speed < MIN_SPEED_CAP || speed > MAX_SPEED_CAP) {
    Serial.print("Warning: Speed ");
    Serial.print(speed);
    Serial.println(" is out of bounds!");
  }
}


// Set motor speeds
void set_motor(int left_speed, int right_speed) {
  // Check speed limits
  check_speed_limits(left_speed);
  check_speed_limits(right_speed);

  Serial.println("Change the motor speed \n");

  // Update motor speeds
  if (left_speed >= MIN_SPEED_CAP && left_speed <= MAX_SPEED_CAP) {
    leftMotor.writeMicroseconds(left_speed);
    Serial.print("Left motor set to: ");
    Serial.println(left_speed);
  }

  if (right_speed >= MIN_SPEED_CAP && right_speed <= MAX_SPEED_CAP) {
    rightMotor.writeMicroseconds(right_speed);
    Serial.print("Right motor set to: ");
    Serial.println(right_speed);
  }
}















