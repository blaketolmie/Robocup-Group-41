#include "test_inc/mid_ir_test.h"
#include "midRangeIR.h"
#include <Arduino.h>

void mid_ir_test() {
  Serial.println("=== Mid IR test: reading smoothed distances ===");
  midRangeIR_init();
  for (;;) {
    midRangeIRLCallback();
    midRangeIRRCallback();

    Serial.print("L:"); Serial.print(getMidRangeIRL());
    Serial.print(" R:"); Serial.println(getMidRangeIRR());
    delay(200);
  }
}
