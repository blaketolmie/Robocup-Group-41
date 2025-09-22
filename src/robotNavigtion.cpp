#include "robotNavigation.h"
#include <Arduino.h>
#include "motors.h"
#include "sensors.h"

int x = 0, y = 0;
float theta = 0;

void initialize_sensors() {
  encoderSetUp();
  motor_init();
  init_sensors();
}

void initialize_position() {
  x = 0;
  y = 0;
  theta = 0;
}

int get_front_distance() {
  // Use average of ultrasonic and ToF for front distance
  sensor_average(); // updates averages internally

  // For simplicity take average of ultrasonic and ToF readings
  float frontDist = (ultrasonicDistance + tofDistance) / 2.0;
  return (int)frontDist;
}

int get_left_distance() {
  sensor_average();

  // Use mid-range IR left and long-range IR left averaged
  float leftDist = (midIRLeft + longIRLeft) / 2.0;
  return (int)leftDist;
}

int get_right_distance() {
  sensor_average();

  // Use mid-range IR right and long-range IR right averaged
  float rightDist = (midIRRight + longIRRight) / 2.0;
  return (int)rightDist;
}

bool is_wall_on_left() {
  return get_left_distance() < MIN_OBSTACLE_DISTANCE;
}

bool is_wall_on_right() {
  return get_right_distance() < MIN_OBSTACLE_DISTANCE;
}

void get_position_from_sensors(int *pos_x, int *pos_y, float *pos_theta) {
  OdomReading();
  *pos_x = (int)GetOdom();
  *pos_y = 0;
  *pos_theta = 0;
}

void move_forward(int speed) {
  MoveForward(speed);
  set_motor();
}

void turn_left(int degrees) {
  TurnLeftForTime(7, degrees * 10);
  set_motor();
}

void turn_right(int degrees) {
  TurnRightForTime(7, degrees * 10);
  set_motor();
}

void follow_wall_left() {
  if (get_left_distance() > MIN_OBSTACLE_DISTANCE) {
    turn_left(10);
  }
  move_forward(6);
}

void follow_wall_right() {
  if (get_right_distance() > MIN_OBSTACLE_DISTANCE) {
    turn_right(10);
  }
  move_forward(6);
}
