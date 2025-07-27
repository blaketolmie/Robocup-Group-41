#include "test_inc/motor_test.h"

// ################################### Motor Test Function ###################################
void motor_test() {
  Serial.println("=== Starting variable speed ramp test ===");
  
  // Start from minimum speed and gradually increase to maximum
  int speed = MIN_SPEED_CAP;
  while (speed <= MAX_SPEED_CAP) {
    Serial.print("Setting speed to: ");
    Serial.println(speed);
    set_motor(speed, speed); // Both motors same speed
    delay(500); // Wait 0.5 seconds between speed changes
    
    // Determine increment based on current speed
    if (speed < LOWER_GAP_VALUE) {
      speed += LOWER_GAP;  // Small increments (10)
      Serial.print("Using LOWER_GAP: +");
      Serial.println(LOWER_GAP);
    } else if (speed > UPPER_GAP_VALUE) {
      speed += UPPER_GAP;  // Large increments (50)
      Serial.print("Using UPPER_GAP: +");
      Serial.println(UPPER_GAP);
    } else {
      speed += NORMAL_GAP; // Normal increments (25)
      Serial.print("Using NORMAL_GAP: +");
      Serial.println(NORMAL_GAP);
    }
  }
  
  Serial.println("Maximum speed reached!");
  delay(2000); // Stay at max speed for 2 seconds
  
  // Gradually slow down back to minimum
  Serial.println("Slowing down...");
  speed = MAX_SPEED_CAP;
  while (speed >= MIN_SPEED_CAP) {
    Serial.print("Setting speed to: ");
    Serial.println(speed);
    set_motor(speed, speed);
    delay(300);
    
    // Determine decrement based on current speed
    if (speed < LOWER_GAP_VALUE) {
      speed -= LOWER_GAP;  // Small decrements (10)
      Serial.print("Using LOWER_GAP: -");
      Serial.println(LOWER_GAP);
    } else if (speed > UPPER_GAP_VALUE) {
      speed -= UPPER_GAP;  // Large decrements (50)
      Serial.print("Using UPPER_GAP: -");
      Serial.println(UPPER_GAP);
    } else {
      speed -= NORMAL_GAP; // Normal decrements (25)
      Serial.print("Using NORMAL_GAP: -");
      Serial.println(NORMAL_GAP);
    }
  }
  
  // Stop motors (outside speed caps)
  Serial.println("Stopping motors (1500 - outside caps)");
  leftMotor.writeMicroseconds(1500);
  rightMotor.writeMicroseconds(1500);
  
  Serial.println("Stopped - waiting 3 seconds before next cycle");
  delay(3000);
}
