#include "test_inc/sensors_test.h"
#include "sensors.h"
#include <Arduino.h>

void sensors_test() {
  Serial.println("=== Sensors test: stubs ===");
  for (;;) {
    // Add real sensor readouts here as they are implemented
    Serial.println("Sensors stub running...");
    delay(500);
  }
}
