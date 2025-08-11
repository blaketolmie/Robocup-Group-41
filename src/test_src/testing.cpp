// ######################################### MOTOR TEST ###########################################

#include "test_inc/testing.h"


void run_tests() {
    Serial.println("Select a test by uncommenting in testing.cpp -> run_tests()");

    // Only enable ONE test at a time. Each test has an infinite loop by design.

    // Motor ramp test
    // motor_test();

    // Long-range IR test
    // long_ir_test();

    // Mid-range IR test
    // mid_ir_test();

    // Ultrasonic test
    // ultrasonic_test();

    // Servo sweep test
    // servo_test();

    // Encoder count test
    // encoder_test();

    // Sensors umbrella test (stubs)
    // sensors_test();

    // System state machine test
    // system_test();

    // Weight collection test
    // weight_collection_test();
}