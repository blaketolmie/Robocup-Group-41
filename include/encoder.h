#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>

//
// Constants for encoder calculations
//
#define COLLECTOR_TICKS_PER_REV 6600
#define ENCODER_MAX_VAL         2147483647
#define ENCODER_MIN_VAL        -2147483648
#define ENCODER_PER_REV         3000
#define DIST_PER_REV            232.5  // mm per revolution
#define ENCODER_PER_DIST        (129.0f / 10.0f)  // = 12.9 ticks/mm

//
// Pin assignments for drive encoders (A and B channels)
//
enum DriveEncoderPinAssignments {
    leftEncoderPinA = 2,
    leftEncoderPinB = 3,
    rightEncoderPinA = 4,
    rightEncoderPinB = 5,
};

//
// Pin assignments for collection encoder
//
enum CollectionMotorPinAssignments {
    collectionEncoderPinA = 32,
    collectionEncoderPinB = 33,
};

//
// PID Constants (externally defined in encoder.cpp)
//
extern const uint16_t Kp; 
extern const uint16_t Ki;
extern const uint16_t Kd;

//
// Encoder position counters (updated in ISR)
//
extern volatile uint16_t PID_leftMotorPos;
extern volatile uint16_t PID_rightMotorPos;

extern volatile unsigned int leftMotorPos;
extern unsigned int prevLeftMotorPos;
extern volatile unsigned int rightMotorPos;
extern unsigned int prevRightMotorPos;

extern volatile int collectionMotorPos;
extern volatile int prevCollectionMotorPos;

//
// Encoder phase state flags for quadrature decoding
//
extern boolean leftASet;
extern boolean leftBSet;
extern boolean rightASet;
extern boolean rightBSet;
extern boolean collectionASet;
extern boolean collectionBSet;

//
// Function Prototypes
//
void LeftEncoderIntHandler();
void RightEncoderIntHandler();
void CollectionEncoderIntHandler();

void InitDriveEncoders();
void InitCollectionEncoder();
void InitEncoders();

// Optional utility functions (if you use them in encoder.cpp)
void UpdateMotorSpeeds();
void UpdateMotorControl();
void ResetEncoders();
void DebugEncoderReadings();

#endif // ENCODER_H
