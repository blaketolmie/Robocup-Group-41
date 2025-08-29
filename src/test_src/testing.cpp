// ######################################### MOTOR TEST ###########################################

#include "test_inc/testing.h"


void run_tests() {
    Serial.println("Select a test by uncommenting in testing.cpp -> run_tests()");

    // Enable both motor and servo tests to run together
    
    static unsigned long lastMotorTest = 0;
    static unsigned long lastServoTest = 0;
    unsigned long now = millis();
    
    // Run motor test every 100ms
    // if (now - lastMotorTest >= 100) {
    //     motor_test();
    //     lastMotorTest = now;
    // }
    
    // Run servo test every 50ms
    if (now - lastServoTest >= 50) {
        servo_test();
        lastServoTest = now;
    }

    // Other tests (commented out for now)
    // long_ir_test();
    // mid_ir_test();
    // ultrasonic_test();
    // encoder_test();
    // sensors_test();
    // system_test();
    // weight_collection_test();
}