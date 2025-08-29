#include "test_inc/weight_collection_test.h"
#include "weight_collection.h"
#include <Arduino.h>

void weight_collection_test() {
  Serial.println("=== Weight collection test ===");
  for (;;) {
    weight_scan();
    collect_weight();
    delay(1000);
  }
}
