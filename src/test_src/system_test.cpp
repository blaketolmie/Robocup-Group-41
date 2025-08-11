#include "test_inc/system_test.h"
#include "system.h"
#include <Arduino.h>

void system_test() {
  Serial.println("=== System test: state transitions ===");
  set_state(STATE_INIT);
  Serial.print("State: "); Serial.println(get_state());
  delay(1000);
  set_state(STATE_AUTO);
  Serial.print("State: "); Serial.println(get_state());
  for(;;) {
    // Could invoke system_callback() manually if needed
    delay(500);
  }
}
