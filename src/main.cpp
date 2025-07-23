#include "Arduino.h"
#include <Servo.h>

// SET THIS TO REAL VALUES
#define LEFT_MOTOR_ADDRESS 0      //Pin corresponding to the left dc motor
#define RIGHT_MOTOR_ADDRESS 1     //Pin corresponding to the right dc motor
// #define MIN_SPEED_CAP 1 //Set the minimum speed value that can be written to the motors
// #define MAX_SPEED_CAP 1 //Set the maximum speed value that can be written to the motors
#define MIN_SPEED_CAP 1050
#define MAX_SPEED_CAP 1950

// Create servo objects for motor control
Servo leftMotor;
Servo rightMotor;


/* Check whether the speed value to be written is within the maximum
 *  and minimum speed caps. Act accordingly.
 *
 */
void check_speed_limits(int speed) {
  if (speed < MIN_SPEED_CAP || speed > MAX_SPEED_CAP) {
    Serial.print("Warning: Speed ");
    Serial.print(speed);
    Serial.println(" is out of bounds!");
  }
}


\
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

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Initializing motors...");
  
  // Attach motors to pins
  leftMotor.attach(LEFT_MOTOR_ADDRESS);
  rightMotor.attach(RIGHT_MOTOR_ADDRESS);
  
  // Start with motors stopped
  set_motor(1500, 1500); // 1500 = stop position
  
  Serial.println("Motors initialized and stopped");
  delay(1000);
}

void loop() {
  Serial.println("=== Starting speed ramp test ===");
  
  // Start from stop and gradually increase speed
  for (int speed = 1500; speed >= 1050; speed -= 25) {
    Serial.print("Setting speed to: ");
    Serial.println(speed);
    set_motor(speed, speed); // Both motors same speed
    delay(500); // Wait 0.5 seconds between speed changes
  }
  
  Serial.println("Maximum forward speed reached!");
  delay(2000); // Stay at max speed for 2 seconds
  
  // Gradually slow down back to stop
  Serial.println("Slowing down...");
  for (int speed = 1050; speed <= 1500; speed += 25) {
    Serial.print("Setting speed to: ");
    Serial.println(speed);
    set_motor(speed, speed);
    delay(300);
  }
  
  Serial.println("Stopped - waiting 3 seconds before next cycle");
  delay(3000);
}




