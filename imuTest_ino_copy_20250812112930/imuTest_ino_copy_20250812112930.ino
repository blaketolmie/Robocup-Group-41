#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(9600);
  delay(2000);

  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055!");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop() {
  // get raw accelerometer data
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print("Accel X: "); Serial.print(accel.x()); Serial.print(" m/s^2, ");
  Serial.print("Accel Y: "); Serial.print(accel.y()); Serial.print(" m/s^2, ");
  Serial.print("Accel Z: "); Serial.print(accel.z()); Serial.println(" m/s^2");

  // raw magnetometer data
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  Serial.print("Mag X: "); Serial.print(mag.x()); Serial.print(" uT, ");
  Serial.print("Mag Y: "); Serial.print(mag.y()); Serial.print(" uT, ");
  Serial.print("Mag Z: "); Serial.print(mag.z()); Serial.println(" uT");

  // raw gyroscope data
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Serial.print("Gyro X: "); Serial.print(gyro.x()); Serial.print(" rad/s, ");
  Serial.print("Gyro Y: "); Serial.print(gyro.y()); Serial.print(" rad/s, ");
  Serial.print("Gyro Z: "); Serial.print(gyro.z()); Serial.println(" rad/s");


  delay(100);
}
