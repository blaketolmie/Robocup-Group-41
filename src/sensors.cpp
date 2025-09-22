//************************************
//         sensors.cpp       
//************************************

// This file contains functions used to read and average
// the sensors.

#include "sensors.h"
#include "Arduino.h"
#include "system.h"
#include "ultrasonic.h"
#include "tof.h"
#include "midRangeIR.h"
#include "longRangeIR.h"

const int proxy1 = 38;

// Timer interval for switching sensors (in milliseconds)
const int sensorSwitchInterval = 1000;  

// Local variables
unsigned long lastSensorSwitchTime = 0;
int sensorIndex = 0; // cycle through sensors

// Sensor data variables
int ultrasonicDistance = 0;
int tofDistance = 0;
float midIRRight = 0;
float midIRLeft = 0;
float longIRRight = 0;
float longIRLeft = 0;

// Read ultrasonic value
void read_ultrasonic(){
  ultrasonicDistance = returnDistance(); // get the latest ultrasonic distance
  Serial.print("Ultrasonic Distance: ");
  Serial.println(ultrasonicDistance);
}

// Read infrared value (mid range)
void read_infrared(){
  midIRRight = getMidRangeIRR();  
  midIRLeft = getMidRangeIRL();   
  Serial.print("Mid-range IR Right: ");
  Serial.println(midIRRight);
  Serial.print("Mid-range IR Left: ");
  Serial.println(midIRLeft);
}

// Read long-range infrared value
void read_longrange_infrared() {
  longIRRight = getLongRangeIRR(); // Get the right long-range IR distance
  longIRLeft = getLongRangeIRL();  // Get the left long-range IR distance
  Serial.print("Long-range IR Right: ");
  Serial.println(longIRRight);
  Serial.print("Long-range IR Left: ");
  Serial.println(longIRLeft);
}

// Read ToF sensor value
void read_tof() {
  tofDistance = distance;  // get the latest ToF distance
  Serial.print("ToF Distance: ");
  Serial.println(tofDistance);
}

float ultrasonicSum = 0;
float infraredSum = 0;
float longRangeIRRightSum = 0; // New sum for long-range IR
float longRangeIRLeftSum = 0;  // New sum for long-range IR
float tofSum = 0;              // New sum for ToF
int readingsCount = 0;

void sensor_average() {
  unsigned long currentTime = millis();
  if (currentTime - lastSensorSwitchTime >= sensorSwitchInterval) {
    lastSensorSwitchTime = currentTime;

    switch (sensorIndex) {
      case 0:
        read_ultrasonic();
        ultrasonicSum += ultrasonicDistance;
        break;
      case 1:
        read_infrared();
        infraredSum += (midIRRight + midIRLeft);  // Averaging two infrared sensors
        break;
      case 2:
        read_longrange_infrared();
        longRangeIRRightSum += longIRRight;
        longRangeIRLeftSum += longIRLeft;
        break;
      case 3:
        read_tof();
        tofSum += tofDistance;
        break;
      default:
        break;
    }

    readingsCount++;
    sensorIndex = (sensorIndex + 1) % 4;
  }

  // Average calculations after several readings
  if (readingsCount > 0) {
    float avgUltrasonic = ultrasonicSum / readingsCount;
    float avgInfrared = infraredSum / readingsCount;
    float avgLongRangeIRRight = longRangeIRRightSum / readingsCount;
    float avgLongRangeIRLeft = longRangeIRLeftSum / readingsCount;
    float avgToF = tofSum / readingsCount;

    // Print the averages for each sensor
    Serial.print("Average Ultrasonic Distance: ");
    Serial.println(avgUltrasonic);
    Serial.print("Average Infrared Distance: ");
    Serial.println(avgInfrared);
    Serial.print("Average Long-range IR Right: ");
    Serial.println(avgLongRangeIRRight);
    Serial.print("Average Long-range IR Left: ");
    Serial.println(avgLongRangeIRLeft);
    Serial.print("Average ToF Distance: ");
    Serial.println(avgToF);
  }

  // Reset sums and readings count after averaging
  ultrasonicSum = 0;
  infraredSum = 0;
  longRangeIRRightSum = 0;
  longRangeIRLeftSum = 0;
  tofSum = 0;
  readingsCount = 0;
}
/* 
float topIRDistance = 100.0;     
float bottomIRDistance = 100.0;  

void updateIRDistances() {
    topIRDistance = getTopIRSensorReading();      // your actual function to get top IR distance
    bottomIRDistance = getBottomIRSensorReading();// your actual function to get bottom IR distance
}

bool check_weight() {
  if (digitalRead(proxy1) == 0) {
    return true;
  }
  return false;
}
  
