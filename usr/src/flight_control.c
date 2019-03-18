//
// Created by HUI Ka Yiu on 2019-03-08.
//

#include "flight_control.h"

/*

void UL_flight_control_pid_controller(UL_flight_control_typedef* flight_control){
    extern UL_PWM_typedef brushless[4];

    flight_control->error[PID_YAW] = - flight_control->IMU->angle_z;
    flight_control->error[PID_PITCH] = - flight_control->IMU->angle_y;
    flight_control->error[PID_ROLL] = - flight_control->IMU->angle_x;

    flight_control->delta_error[PID_YAW] = - flight_control->previous_error[PID_YAW];
    flight_control->delta_error[PID_PITCH] = - flight_control->previous_error[PID_PITCH];
    flight_control->delta_error[PID_ROLL] = - flight_control->previous_error[PID_ROLL];

    flight_control->error_sum[PID_YAW] += flight_control->error[PID_YAW];
    flight_control->error_sum[PID_PITCH] += flight_control->error[PID_PITCH];
    flight_control->error_sum[PID_ROLL] += flight_control->error[PID_YAW];

    flight_control->previous_error[PID_YAW] = flight_control->error[PID_YAW];
    flight_control->previous_error[PID_PITCH] = flight_control->error[PID_PITCH];
    flight_control->previous_error[PID_ROLL] = flight_control->error[PID_ROLL];

    flight_control->pid[PID_YAW] = (flight_control->error[PID_YAW] * KP_YAW) + (flight_control->error_sum[PID_YAW] * KI_YAW)
                                   + (flight_control->delta_error[PID_YAW] * KD_YAW);
    flight_control->pid[PID_PITCH] = (flight_control->error[PID_PITCH] * KP_PITCH) + (flight_control->error_sum[PID_PITCH] * KI_PITCH)
                                   + (flight_control->delta_error[PID_PITCH] * KD_PITCH);
    flight_control->pid[PID_ROLL] = (flight_control->error[PID_ROLL] * KP_ROLL) + (flight_control->error_sum[PID_ROLL] * KI_ROLL)
                                   + (flight_control->delta_error[PID_ROLL] * KD_ROLL);

    UL_PWM_SetUs(&brushless[0], + flight_control[PID_ROLL] + flight_control[PID_PITCH] - flight_control[PID_YAW]);
    UL_PWM_SetUs(&brushless[1], - flight_control[PID_ROLL] + flight_control[PID_PITCH] + flight_control[PID_YAW]);
    UL_PWM_SetUs(&brushless[2], + flight_control[PID_ROLL] - flight_control[PID_PITCH] + flight_control[PID_YAW]);
    UL_PWM_SetUs(&brushless[3], - flight_control[PID_ROLL] - flight_control[PID_PITCH] - flight_control[PID_YAW]);

}
*/