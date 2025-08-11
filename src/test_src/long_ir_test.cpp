#include "test_inc/long_ir_test.h"
#include "longRangeIR.h"
#include <Arduino.h>

void long_ir_test() {
  Serial.println("=== Long IR test: reading smoothed distances ===");
  longRangeIR_init();
  for (;;) {
    // Update filters and mapping
    longRangeIRLCallback();
    longRangeIRRCallback();

    // Read values
    Serial.print("L:"); Serial.print(getLongRangeIRL());
    Serial.print(" R:"); Serial.println(getLongRangeIRR());
    delay(200);
  }
}
