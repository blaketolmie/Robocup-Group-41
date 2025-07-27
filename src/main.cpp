#include "globals.h"
#include "test_inc/testing.h"
#include "motors.h"

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Beginning robot initialization...");
  motors_init(); // Initialize motors
  
  Serial.println("Robot initialization complete.");
  delay(1000);
}

void loop() {
  // Comment out when running other stuff since run_tests() has its own unbreakable while loops
  run_tests(); // Run all tests




}




