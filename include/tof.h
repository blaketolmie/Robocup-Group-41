#ifndef TOF_TASK_H
#define TOF_TASK_H

#define _TASK_MICRO_RES
#include <TaskSchedulerDeclarations.h>
#include <Adafruit_VL53L0X.h>
#include "Arduino.h"

extern Task tTof; // TaskScheduler task for the TOF sensor

extern Adafruit_VL53L0X lox; // ToF sensor object

extern VL53L0X_RangingMeasurementData_t measure; 

extern int distance;   
extern int prevDistance;                         
extern int count;       
extern int threshold;
extern int maxDistance; 
extern int minDistance;   

bool tof_init(); // Initializes the ToF sensor
void tofTaskCallback(); // Task callback function to read and process distance measurements

#endif // TOF_TASK_H