// Moved from main.cpp
#ifndef MOTORS_H_
#define MOTORS_H_

#include <globals.h>

// SET THIS TO REAL VALUES
#define MIN_SPEED_CAP 1600
#define MAX_SPEED_CAP 2005

// Create servo objects for motor control
extern Servo leftMotor;
extern Servo rightMotor;

void motors_init();
void check_speed_limits(int speed);
void set_motor(int left_speed, int right_speed);




#endif /* MOTORS_H_ */