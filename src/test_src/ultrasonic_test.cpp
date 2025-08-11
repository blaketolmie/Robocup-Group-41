#include "test_inc/ultrasonic_test.h"
#include "ultrasonic.h"
#include <Arduino.h>
#include <TaskSchedulerDeclarations.h>

extern Task tUltrasonic;

void ultrasonic_test() {
  Serial.println("=== Ultrasonic test: distance (cm) ===");
  ultrasonic_init();
  tUltrasonic.enableIfNot();
  for (;;) {
    Serial.print("D:"); Serial.println(returnDistance());
    delay(200);
  }
}
