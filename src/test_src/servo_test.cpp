#include "test_inc/servo_test.h"
#include "myServo.h"
#include <Arduino.h>
#include <TaskSchedulerDeclarations.h>

extern Task tServo;

void servo_test() {
  Serial.println("=== Servo test: sweeping in/out ===");
  myservo_init();
  set_servo_velocity(50);
  for (;;) {
    servo_out();
    tServo.enableIfNot();
    while (!is_servo_out()) { delay(10); }
    delay(500);
    servo_in();
    tServo.enableIfNot();
    while (!is_servo_in()) { delay(10); }
    delay(500);
  }
}
