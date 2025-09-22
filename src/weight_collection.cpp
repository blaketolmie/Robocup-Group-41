/************************************
 *        weight_collection.cpp       *
 *************************************/

#include "weight_collection.h"
#include "Arduino.h"
#include "myServo.h"  

// Uncomment in sensors.cpp
extern float topIRDistance;
extern float bottomIRDistance;

// Change these 
const float WALL_THRESHOLD = 0.0;      // Distance (cm) below which something is detected
const float WEIGHT_TOP_THRESHOLD = 0.0;  // Top sensor distance threshold to differentiate weight from wall
const float WEIGHT_BOTTOM_THRESHOLD = 0.0; // Bottom sensor threshold for detecting weight presence

// Avoid multiple collections at once
static bool weightBeingCollected = false;

void weight_scan()
{
  // Detect weight by difference in IR sensor readings
  bool topBlocked = topIRDistance < WEIGHT_TOP_THRESHOLD;
  bool bottomBlocked = bottomIRDistance < WEIGHT_BOTTOM_THRESHOLD;

  if (bottomBlocked && !topBlocked && !weightBeingCollected) {
    Serial.println("Weight detected! Starting collection...");
    weightBeingCollected = true;
    collect_weight();
  }
}

void collect_weight()
{
  // Move servo out to collect
  servo_out();

  // Wait for servo to finish moving out
  while (is_servo_activated()) {
    delay(10);
  }

  delay(1000); // Hold for a moment

  // Retract servo
  servo_in();

  // Wait for servo to finish retracting
  while (is_servo_activated()) {
    delay(10);
  }

  Serial.println("Weight collected.");
  weightBeingCollected = false;
}

