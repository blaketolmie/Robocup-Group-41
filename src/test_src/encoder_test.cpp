#include "test_inc/encoder_test.h"
#include "encoder.h"
#include <Arduino.h>

void encoder_test() {
  Serial.println("=== Encoder test: printing counts ===");
  InitEncoders();
  for (;;) {
    noInterrupts();
    unsigned int l = leftMotorPos;
    unsigned int r = rightMotorPos;
    int c = collectionMotorPos;
    interrupts();
    Serial.print("L:"); Serial.print(l);
    Serial.print(" R:"); Serial.print(r);
    Serial.print(" C:"); Serial.println(c);
    delay(100);
  }
}
