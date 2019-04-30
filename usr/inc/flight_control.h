//
// Created by HUI Ka Yiu on 2019-03-08.
//

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
#define KP_PITCH 1 //0.001
#define KI_PITCH 0.001
#define KD_PITCH 10
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

#define MODE_AIR 0
#define MODE_LAND_F 1
#define MODE_LAND_B 2

extern UL_IMU_typedef IMU;
extern UL_RC_typedef RC;
extern UL_PWM_typedef brushless[4];
extern UL_PWM_typedef servo[4];
extern UL_TFT_typedef Tft;

typedef struct {
    uint8_t ready, currentMode;
    uint32_t output[4];
    double target[3], error[3], error_sum[3], delta_error[3], previous_error[3], pid[3];
    double speed_x, speed_y, speed_z;
    double displacement_x, displacement_y, displacement_z;

}UL_flight_control_typedef;

void UL_flight_control_init(UL_flight_control_typedef* flightControl);

void UL_flight_control_print(UL_flight_control_typedef* flight_control);

void UL_flight_control_air_pid_controller(UL_flight_control_typedef* flight_control);

void UL_flight_control_land_pid_controller(UL_flight_control_typedef* flight_control);

void UL_flight_control(UL_flight_control_typedef* flight_control);

#endif //PLANE_FLIGHT_CONTROL_H
