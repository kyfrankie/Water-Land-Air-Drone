//
// Created by HUI Ka Yiu on 2019-03-08.
//

#include "flight_control.h"



void UL_flight_control_init(UL_flight_control_typedef* flightControl){
    for (int i = 0; i < 3; i++) {
        flightControl->error[i] = 0;
        flightControl->delta_error[i] = 0;
        flightControl->error_sum[i] = 0;
        flightControl->pid[i] = 0;
        flightControl->previous_error[i] = 0;
    }
    flightControl->displacement_x = 0;
    flightControl->displacement_y = 0;
    flightControl->displacement_z = 0;
    flightControl->speed_x = 0;
    flightControl->speed_y = 0;
    flightControl->speed_z = 0;
}

void UL_flight_control_pid_controller(UL_flight_control_typedef* flight_control){

    flight_control->error[PID_YAW] = RC.yaw  - IMU.yaw;        //! limited angle to 45degree each side
    flight_control->error[PID_PITCH] = RC.pitch - IMU.pitch;
    flight_control->error[PID_ROLL] = RC.roll - IMU.roll;

    flight_control->delta_error[PID_YAW] = RC.yaw - flight_control->previous_error[PID_YAW];
    flight_control->delta_error[PID_PITCH] = RC.pitch - flight_control->previous_error[PID_PITCH];
    flight_control->delta_error[PID_ROLL] = RC.roll - flight_control->previous_error[PID_ROLL];

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

    UL_PWM_SetUs(&brushless[0], RC.throttle + flight_control->pid[PID_ROLL] + flight_control->pid[PID_PITCH] - flight_control->pid[PID_YAW]);
    UL_PWM_SetUs(&brushless[1], RC.throttle - flight_control->pid[PID_ROLL] + flight_control->pid[PID_PITCH] + flight_control->pid[PID_YAW]);
    UL_PWM_SetUs(&brushless[2], RC.throttle + flight_control->pid[PID_ROLL] - flight_control->pid[PID_PITCH] + flight_control->pid[PID_YAW]);
    UL_PWM_SetUs(&brushless[3], RC.throttle - flight_control->pid[PID_ROLL] - flight_control->pid[PID_PITCH] - flight_control->pid[PID_YAW]);

}

void UL_flight_control_print(UL_flight_control_typedef* flight_control){
    UL_TFT_ST7735_WriteString(&Tft, 25, 2, "Y", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteString(&Tft, 50, 2, "P", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteString(&Tft, 75, 2, "R", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteString(&Tft, 100, 2, "T", FONT_7x10, ST7735_WHITE, ST7735_BLACK);

    UL_TFT_ST7735_WriteString(&Tft, 1, 20, "RC", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 25, 20, RC.yaw, FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 50, 20, RC.pitch, FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 75, 20, RC.roll, FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 100, 20, RC.throttle, FONT_7x10, ST7735_WHITE, ST7735_BLACK);

    UL_TFT_ST7735_WriteString(&Tft, 1, 40, "IMU", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 25, 40, IMU.roll, FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 50, 40, IMU.pitch, FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 75, 40, IMU.roll, FONT_7x10, ST7735_WHITE, ST7735_BLACK);

    UL_TFT_ST7735_WriteString(&Tft, 1, 60, "ESC", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 25, 60, RC.throttle + flight_control->pid[PID_ROLL] + flight_control->pid[PID_PITCH] - flight_control->pid[PID_YAW], FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 50, 60, RC.throttle - flight_control->pid[PID_ROLL] + flight_control->pid[PID_PITCH] + flight_control->pid[PID_YAW], FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 75, 60, RC.throttle + flight_control->pid[PID_ROLL] - flight_control->pid[PID_PITCH] + flight_control->pid[PID_YAW], FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 100, 60, RC.throttle - flight_control->pid[PID_ROLL] - flight_control->pid[PID_PITCH] - flight_control->pid[PID_YAW], FONT_7x10, ST7735_WHITE, ST7735_BLACK);
}