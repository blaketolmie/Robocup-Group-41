#include "test_inc/servo_test.h"
#include "myServo.h"
#include <Arduino.h>

void servo_test() {
    Serial.println("=== Simple Servo Sweep Test ===");
    
    // Initialize servo
    myservo_init();
    set_servo_velocity(30); // Set moderate speed
    
    static unsigned long lastUpdate = 0;
    static unsigned long lastAction = 0;
    static int testState = 0; // 0=out, 1=wait_out, 2=in, 3=wait_in
    static bool testInitialized = false;
    
    // Initialize test
    if (!testInitialized) {
        servo_out();
        lastAction = millis();
        testState = 0;
        testInitialized = true;
        Serial.println("Starting servo sweep test...");
    }
    
    unsigned long now = millis();
    
    // Update servo position regularly (acts like a mini scheduler)
    if (now - lastUpdate >= 50) { // Update every 50ms
        myservo_callback();
        lastUpdate = now;
    }
    
    // State machine for test sequence
    switch (testState) {
        case 0: // Moving out
            if (is_servo_out()) {
                Serial.println("✓ Servo reached OUT position");
                lastAction = now;
                testState = 1;
            }
            break;
            
        case 1: // Wait at out position
            if (now - lastAction > 1000) { // Wait 1 second
                Serial.println("Moving servo IN...");
                servo_in();
                testState = 2;
            }
            break;
            
        case 2: // Moving in
            if (is_servo_in()) {
                Serial.println("✓ Servo reached IN position");
                lastAction = now;
                testState = 3;
            }
            break;
            
        case 3: // Wait at in position
            if (now - lastAction > 1000) { // Wait 1 second
                Serial.println("Moving servo OUT...");
                servo_out();
                testState = 0;
                Serial.println("--- Cycle complete ---");
            }
            break;
    }
    
    // Small delay to prevent overwhelming the loop
    delay(10);
}
//   }
// }

// bool servo_enable_callback(){
//   isServoActivated = true;
//   return true;
// }

// void servo_disable_callback(){
//   isServoActivated = false;
// }


// void myservo_init(){
//   myservo_left.attach(servo_left_pin);  // attaches the servo  to the servo object
//   myservo_right.attach(servo_right_pin);  // attaches the servo  to the servo object
//   move_servo(pos);
//   tServo.setOnEnable(&servo_enable_callback);
//   tServo.setOnDisable(&servo_disable_callback);
// }

