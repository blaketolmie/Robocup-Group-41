#include "encoder.h"

volatile uint16_t PID_leftMotorPos = 0;
volatile uint16_t PID_rightMotorPos = 0;

const uint16_t Kp = 1;
const uint16_t Ki = 0;
const uint16_t Kd = 0;

volatile unsigned int leftMotorPos = 0;
unsigned int prevLeftMotorPos = 1;
volatile unsigned int rightMotorPos = 0;
unsigned int prevRightMotorPos = 1;

volatile int collectionMotorPos = 0;
volatile int prevCollectionMotorPos = 1;

#define DEBOUNCE_DELAY 5  // Debounce delay for encoder signals (in milliseconds)
#define ENCODER_MAX_VALUE 65535  // Max value for encoder (16-bit encoder overflow)
// Timers for debouncing
unsigned long lastLeftEncoderInterrupt = 0;
unsigned long lastRightEncoderInterrupt = 0;
unsigned long lastCollectionEncoderInterrupt = 0;

// Encoder state flags
boolean leftASet = false;
boolean leftBSet = false;
boolean rightASet = false;
boolean rightBSet = false;
boolean collectionASet = false;
boolean collectionBSet = false;

// Left drive motor encoder interrupt handler
void LeftEncoderIntHandler() {
    if (millis() - lastLeftEncoderInterrupt > DEBOUNCE_DELAY) {
        lastLeftEncoderInterrupt = millis();
        
        leftASet = digitalRead(leftEncoderPinA) == HIGH;
        leftMotorPos += (leftASet != leftBSet) ? -1 : +1;

        leftBSet = digitalRead(leftEncoderPinB) == HIGH;
        leftMotorPos += (leftASet == leftBSet) ? -1 : +1;

        // Handle overflow for left encoder
        if (leftMotorPos > ENCODER_MAX_VALUE) {
            leftMotorPos = 0;
        }
        if (leftMotorPos < 0) {
            leftMotorPos = ENCODER_MAX_VALUE;
        }
    }
}

// Right drive motor encoder interrupt handler
void RightEncoderIntHandler() {
    if (millis() - lastRightEncoderInterrupt > DEBOUNCE_DELAY) {
        lastRightEncoderInterrupt = millis();
        
        rightASet = digitalRead(rightEncoderPinA) == HIGH;
        rightMotorPos += (rightASet != rightBSet) ? +1 : -1;

        rightBSet = digitalRead(rightEncoderPinB) == HIGH;
        rightMotorPos += (rightASet == rightBSet) ? +1 : -1;

        // Handle overflow for right encoder
        if (rightMotorPos > ENCODER_MAX_VALUE) {
            rightMotorPos = 0;
        }
        if (rightMotorPos < 0) {
            rightMotorPos = ENCODER_MAX_VALUE;
        }
    }
}

// Collection motor encoder interrupt handler
void CollectionEncoderIntHandler() {
    if (millis() - lastCollectionEncoderInterrupt > DEBOUNCE_DELAY) {
        lastCollectionEncoderInterrupt = millis();
        
        collectionASet = digitalRead(collectionEncoderPinA) == HIGH;
        collectionMotorPos += (collectionASet != collectionBSet) ? +1 : -1;

        collectionBSet = digitalRead(collectionEncoderPinB) == HIGH;
        collectionMotorPos += (collectionASet == collectionBSet) ? +1 : -1;

        // Handle overflow for collection motor encoder
        if (collectionMotorPos > ENCODER_MAX_VALUE) {
            collectionMotorPos = 0;
        }
        if (collectionMotorPos < 0) {
            collectionMotorPos = ENCODER_MAX_VALUE;
        }
    }
}

/**
 * Initialise the drive motor encoders
 */
void InitDriveEncoders() {
    pinMode(leftEncoderPinA, INPUT);  // Set encoder pins as inputs
    pinMode(leftEncoderPinB, INPUT);
    pinMode(rightEncoderPinA, INPUT);
    pinMode(rightEncoderPinB, INPUT);

    attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), LeftEncoderIntHandler, CHANGE);  // Set up an interrupt for each encoder
    attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), RightEncoderIntHandler, CHANGE);
}

/**
 * Initialise the collection motor encoder
 */
void InitCollectionEncoder() {
    pinMode(collectionEncoderPinA, INPUT);
    pinMode(collectionEncoderPinB, INPUT);

    attachInterrupt(digitalPinToInterrupt(collectionEncoderPinA), CollectionEncoderIntHandler, CHANGE);
}

/**
 * Initialise all encoders
 */
void InitEncoders() {
    InitDriveEncoders();
    InitCollectionEncoder();
}

/**
 * Update motor speeds based on encoder readings
 * Calculates the speed by measuring the change in position over time
 */
void UpdateMotorSpeeds() {
    unsigned long currentTime = millis();
    static unsigned long lastUpdateTime = 0;

    if (currentTime - lastUpdateTime >= 100) {  // Update every 100 ms
        unsigned int leftSpeed = leftMotorPos - prevLeftMotorPos;
        unsigned int rightSpeed = rightMotorPos - prevRightMotorPos;

        // Store current positions for the next speed calculation
        prevLeftMotorPos = leftMotorPos;
        prevRightMotorPos = rightMotorPos;

        // Print the speed to Serial Monitor for debugging
        Serial.print("Left Speed: ");
        Serial.print(leftSpeed);
        Serial.print(", Right Speed: ");
        Serial.println(rightSpeed);

        lastUpdateTime = currentTime;
    }
}

/**
 * Reset all encoder positions to zero (useful at the start of a new task)
 */
void ResetEncoders() {
    leftMotorPos = 0;
    rightMotorPos = 0;
    collectionMotorPos = 0;
}

/**
 * Debug function to print encoder positions to the serial monitor
 */
void DebugEncoderReadings() {
    Serial.print("Left Motor Position: ");
    Serial.println(leftMotorPos);
    Serial.print("Right Motor Position: ");
    Serial.println(rightMotorPos);
    Serial.print("Collection Motor Position: ");
    Serial.println(collectionMotorPos);
}

/**
 * PID Controller for motor control
 * Uses the encoder position to adjust the motor's speed in a closed-loop manner
 */
void UpdateMotorControl() {
    int targetLeftSpeed = 100;   // Target speed (encoder counts per second)
    int targetRightSpeed = 100;  // Target speed (encoder counts per second)

    static int prevLeftError = 0;
    static int prevRightError = 0;

    // Calculate speed error
    int leftMotorError = targetLeftSpeed - (leftMotorPos - prevLeftMotorPos);
    int rightMotorError = targetRightSpeed - (rightMotorPos - prevRightMotorPos);

    // Apply PID formula
    int leftMotorOutput = Kp * leftMotorError + Ki * (leftMotorError + prevLeftError) + Kd * (leftMotorError - prevLeftError);
    int rightMotorOutput = Kp * rightMotorError + Ki * (rightMotorError + prevRightError) + Kd * (rightMotorError - prevRightError);

    // Store previous errors for next PID calculation
    prevLeftError = leftMotorError;
    prevRightError = rightMotorError;

    