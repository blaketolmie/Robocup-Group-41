#define _TASK_MICRO_RES
#include <TaskSchedulerDeclarations.h>
#include "tof.h"
#include "Arduino.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

Task tTof(100*TASK_MILLISECOND, -1, &tofTaskCallback); // updates every 100ms
VL53L0X_RangingMeasurementData_t measure;
int distance = 0;   // distance in mm   
int prevDistance = 0; // previous distance in mm
int count = 0;       // counter for filtering
int threshold = 50; // threshold in mm
int maxDistance = 2000; // maximum distance in mm
int minDistance = 50;   // minimum distance in mm

bool tof_init() {
  if (!lox.begin()) {
    //Serial.println(F("Failed to boot VL53L0X"));
    return false;
  }
  //Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
  return true;
}
void tofTaskCallback() {
  prevDistance = distance;
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // 4 is out of range
    distance = measure.RangeMilliMeter;
    if (distance > maxDistance || distance < minDistance) { // out of bounds
      distance = prevDistance;
    } else if (abs(distance - prevDistance) > threshold) { // sudden jump
      count++; 
      if (count == 5) { // if 5 consecutive readings are out of bounds, accept the new reading
        count = 0;
      } else {
        distance = prevDistance;  // otherwise, keep the previous distance
      }
    } else {
      count = 0; // reset count if the reading is stable
    }
  } else {
    distance = prevDistance; // keep previous distance if measurement failed
  }
  //Serial.print("Distance (mm): "); Serial.println(distance);
}