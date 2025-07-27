#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include "../../include/globals.h"
#include "../../include/motors.h"

void motor_test();

// ############################## Testing Variables ##############################
// Speed increment variables
#define LOWER_GAP 50    // Small increments below 1700
#define LOWER_GAP_VALUE 1700
#define UPPER_GAP 2    // Large increments above 1900
#define UPPER_GAP_VALUE 1990
#define NORMAL_GAP 25   // Normal increments between 1700-1900


#endif // MOTOR_TEST_H