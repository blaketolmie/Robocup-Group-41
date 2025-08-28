#include "test_inc/tof_test.h"
#include "tof.h"
#include <Arduino.h>

void tof_test() {
  Serial.println("=== ToF test: reading smoothed distances ===");
  if (!tof_init()) {
    Serial.println("Failed to initialize ToF sensor!");
    return;
  }
  for (;;) {
    tofTaskCallback();
    Serial.print("Distance (mm): "); Serial.println(distance);
    delay(200);
  }
}