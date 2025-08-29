#include "test_inc/motor_test.h"

// ################################### Motor Test Function ###################################
void motor_test() {
    static int speed = MIN_SPEED_CAP;
    static unsigned long lastSpeedChange = 0;
    static bool goingUp = true;
    static bool testInitialized = false;
    
    // Initialize test
    if (!testInitialized) {
        Serial.println("=== Starting variable speed ramp test ===");
        speed = MIN_SPEED_CAP;
        goingUp = true;
        lastSpeedChange = millis();
        testInitialized = true;
    }
    
    unsigned long now = millis();
    
    // Change speed every 500ms
    if (now - lastSpeedChange >= 500) {
        if (goingUp) {
            // Going up
            Serial.print("Setting speed to: ");
            Serial.println(speed);
            set_motor(speed, speed); // Both motors same speed
            
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
            
            // Check if we've reached maximum
            if (speed >= MAX_SPEED_CAP) {
                Serial.println("Maximum speed reached! Starting to slow down...");
                goingUp = false;
                speed = MAX_SPEED_CAP;
            }
        } else {
            // Going down
            Serial.print("Setting speed to: ");
            Serial.println(speed);
            set_motor(speed, speed);
            
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
            
            // Check if we've reached minimum
            if (speed <= MIN_SPEED_CAP) {
                Serial.println("Minimum speed reached! Stopping motors...");
                leftMotor.writeMicroseconds(1500);
                rightMotor.writeMicroseconds(1500);
                Serial.println("--- Motor cycle complete, restarting ---");
                goingUp = true;
                speed = MIN_SPEED_CAP;
            }
        }
        
        lastSpeedChange = now;
    }
}
