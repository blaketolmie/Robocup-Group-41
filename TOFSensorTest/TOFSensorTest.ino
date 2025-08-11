#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

void loop() {
  uint16_t distance = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred()) {
    Serial.println("TIMEOUT");
  } else {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");

    
    if (distance < 1000) {
      Serial.println("Object detected!");
    } else {
      Serial.println("No object nearby.");
    }
  }

  delay(100);  
}