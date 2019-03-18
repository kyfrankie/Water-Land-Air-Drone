//
// Created by HUI Ka Yiu on 2019-03-08.
//
/*

#ifndef PLANE_FLIGHT_CONTROL_H
#define PLANE_FLIGHT_CONTROL_H

#include "main.h"


#define time_interval_ms 50

#define PI (float)(3.1415)

#define PID_THROTTLE 0
#define KP_THROTTLE 0
#define KI_THROTTLE 0
#define KD_THROTTLE 0
#define MIN_THROTTLE (-100)
#define MAX_THROTTLE (100)

#define PID_PITCH 0
#define KP_PITCH 0
#define KI_PITCH 0
#define KD_PITCH 0
#define MIN_PITCH (-PI/6)
#define MAX_PITCH (PI/6)

#define PID_ROLL 1
#define KP_ROLL 0
#define KI_ROLL 0
#define KD_ROLL 0
#define MIN_ROLL (-PI/6)
#define MAX_ROLL (PI/6)

#define PID_YAW 2
#define KP_YAW 0
#define KI_YAW 0
#define KD_YAW 0
#define MIN_YAW (-PI)
#define MAX_YAW (PI)

typedef struct {
    UL_IMU_typedef* IMU;
    double error[3] = {0,0,0};
    double error_sum[3] = {0,0,0};
    double delta_error[3] = {0,0,0};
    double previous_error[3] = {0,0,0};
    double pid[3] = {0,0,0};
    double speed_x, speed_y, speed_z;
    double displacement_x, displacement_y, displacement_z;

}UL_flight_control_typedef;


void UL_flight_control_pid_controller(UL_flight_control_typedef* flight_control, );
#endif //PLANE_FLIGHT_CONTROL_H
*/