#ifndef ROBOT_NAVIGATION_H
#define ROBOT_NAVIGATION_H

#define MIN_OBSTACLE_DISTANCE 30  // cm

void initialize_sensors();
void initialize_position();
void move_forward(int speed);
void turn_left(int degrees);
void turn_right(int degrees);

int get_front_distance();
int get_left_distance();
int get_right_distance();

bool is_wall_on_left();
bool is_wall_on_right();

void follow_wall_left();
void follow_wall_right();

void get_position_from_sensors(int *x, int *y, float *theta);

#endif
