#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Robot state
float posX = 0;
float posY = 0;
float velX = 0;
float velY = 0;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055!");
    while (1);
  }

  bno.setExtCrystalUse(true);
  Serial.println("IMU Initialized.");
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; 
  lastTime = currentTime;

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);


  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float headingDeg = euler.x();
  float headingRad = headingDeg * PI / 180.0;

  float ax = accel.x();
  float ay = accel.y();

  float accX_world = ax * cos(headingRad) - ay * sin(headingRad);
  float accY_world = ax * sin(headingRad) + ay * cos(headingRad);

  // Integrate acceleration to get velocity
  velX += accX_world * dt;
  velY += accY_world * dt;

  // Integrate velocity to get position
  posX += velX * dt;
  posY += velY * dt;

  // Output estimated position
  Serial.print("X: ");
  Serial.print(posX, 3);
  Serial.print(" m, Y: ");
  Serial.print(posY, 3);
  Serial.print(" m, Heading: ");
  Serial.print(headingDeg, 1);
  Serial.println("°");

  delay (100)

}
