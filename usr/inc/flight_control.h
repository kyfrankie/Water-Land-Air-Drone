//
// Created by HUI Ka Yiu on 2019-03-08.
//

#ifndef PLANE_FLIGHT_CONTROL_H
#define PLANE_FLIGHT_CONTROL_H

#include "main.h"


#define time_interval_ms 50

#define PI (float)(3.1415)
#define MAXI 1000.0

#define PID_PITCH 0
#define KP_PITCH 6        //9, 8.5, 8.8. 8.9 8
#define KI_PITCH 0.008        //0   0.005
#define KD_PITCH 25         //15  8 19 18 15 18 20

#define PID_ROLL 1
#define KP_ROLL 6           //3,6, 8, 4
#define KI_ROLL 0.004       //0.002
#define KD_ROLL 22          //4,6, 15, 20

#define PID_YAW 2
#define KP_YAW 2              //0,1
#define KI_YAW 0
#define KD_YAW 1

#define PID_ALL 3
#define KP_ALL 0           //0,1
#define KI_ALL 0
#define KD_ALL 0

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
    double target[3], error[4], error_sum[4], delta_error[4], previous_error[4], pid[4];
    double speed_x, speed_y, speed_z;
    //double temp_imu[200], temp_err[200], temp_delta[200], temp_sum[200], temp_pid[200], temp_out[200], temp_throttle[200];
    //uint32_t output0[200], output1[200], output2[200], output3[200];
    uint8_t index;
    double displacement_x, displacement_y, displacement_z;

}UL_flight_control_typedef;

void UL_flight_control_init(UL_flight_control_typedef* flightControl);

void UL_flight_control_print(UL_flight_control_typedef* flight_control);

void UL_flight_control_air_pid_controller(UL_flight_control_typedef* flight_control);

void UL_flight_control_land_pid_controller(UL_flight_control_typedef* flight_control);

void UL_flight_control(UL_flight_control_typedef* flight_control);

#endif //PLANE_FLIGHT_CONTROL_H
