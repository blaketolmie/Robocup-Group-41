//************************************
//         sensors.h     
//************************************

#ifndef SENSORS_H_
#define SENSORS_H_

// Function declarations for reading sensors
void read_ultrasonic();
void read_infrared();
void read_longrange_infrared();
void read_tof();
//void read_colour()

// Pass in data and average the lot
void sensor_average(/* Parameters */);
//bool check_weight();


#endif /* SENSORS_H_ */
