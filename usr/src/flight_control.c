//
// Created by HUI Ka Yiu on 2019-03-08.
//

#include "flight_control.h"
#include "stdlib.h"


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

    flightControl->target[PID_YAW] = IMU.yaw;

    flightControl->ready = 1;
}

void UL_flight_control(UL_flight_control_typedef* flight_control){
    if (RC.diffCapture[RC_MODE] < 1050){
        if (flight_control->currentMode != MODE_LAND_F) {
            flight_control->currentMode = MODE_LAND_F;
            UL_PWM_SetUs(&servo[0], 1000);
            UL_PWM_SetUs(&servo[1], 2000);
            UL_PWM_SetUs(&servo[2], 1000);
            UL_PWM_SetUs(&servo[3], 2000);
        }
        if (RC.diffCapture[RC_THROTTLE] > 1050)
            UL_flight_control_land_pid_controller(flight_control);
    }
    else if (RC.diffCapture[RC_MODE] > 1450 && RC.diffCapture[RC_MODE] < 1550) {
        if (flight_control->currentMode != MODE_AIR){
            flight_control->currentMode = MODE_AIR;
            UL_PWM_SetUs(&servo[0], 1550);
            UL_PWM_SetUs(&servo[1], 1500);
            UL_PWM_SetUs(&servo[2], 1550);
            UL_PWM_SetUs(&servo[3], 1500);
        }
        if (RC.diffCapture[RC_THROTTLE] > 1050)
            UL_flight_control_air_pid_controller(flight_control);
    }
    else if (RC.diffCapture[RC_MODE] > 1950){
        if (flight_control->currentMode != MODE_LAND_B){
            flight_control->currentMode = MODE_LAND_B;
            UL_PWM_SetUs(&servo[0], 2000);
            UL_PWM_SetUs(&servo[1], 1000);
            UL_PWM_SetUs(&servo[2], 2000);
            UL_PWM_SetUs(&servo[3], 1000);
        }
        if (RC.diffCapture[RC_THROTTLE] > 1050)
            UL_flight_control_land_pid_controller(flight_control);
    }
}

//! AIR PID
void UL_flight_control_air_pid_controller(UL_flight_control_typedef* flight_control){

    if (RC.diffCapture[RC_YAW]/10-150 > 0 || RC.diffCapture[RC_YAW]/10-150 < 0 ) {
        double temp = flight_control->target[PID_YAW] + (RC.diffCapture[RC_YAW] / 10 - 150) / 50.0 * 5.0;
        if (temp > 360.0)
            flight_control->target[PID_YAW] = temp - 360.0;
        else if (temp < 0.0)
            flight_control->target[PID_YAW] = temp + 360.0;
        else flight_control->target[PID_YAW] = temp;
    }

    flight_control->target[PID_PITCH] = (RC.diffCapture[RC_PITCH]/10-150)/50.0 * 30.0;  //! limited to 30 degree
    flight_control->target[PID_ROLL] = (RC.diffCapture[RC_ROLL]/10-150)/50.0 * 30.0;

    flight_control->error[PID_YAW] = flight_control->target[PID_YAW] - IMU.yaw;
    flight_control->error[PID_PITCH] = flight_control->target[PID_PITCH] - IMU.pitch;
    flight_control->error[PID_ROLL] = flight_control->target[PID_ROLL] - IMU.roll;


    flight_control->delta_error[PID_YAW] = flight_control->error[PID_YAW] - flight_control->previous_error[PID_YAW];
    flight_control->delta_error[PID_PITCH] = flight_control->error[PID_PITCH] - flight_control->previous_error[PID_PITCH];
    flight_control->delta_error[PID_ROLL] = flight_control->error[PID_ROLL] - flight_control->previous_error[PID_ROLL];

    flight_control->error_sum[PID_YAW] += flight_control->error[PID_YAW];
    flight_control->error_sum[PID_PITCH] += flight_control->error[PID_PITCH];
    flight_control->error_sum[PID_ROLL] += flight_control->error[PID_YAW];

    flight_control->previous_error[PID_YAW] = flight_control->error[PID_YAW];
    flight_control->previous_error[PID_PITCH] = flight_control->error[PID_PITCH];
    flight_control->previous_error[PID_ROLL] = flight_control->error[PID_ROLL];

    flight_control->pid[PID_YAW] = (flight_control->error[PID_YAW] * KP_YAW) + (flight_control->error_sum[PID_YAW] * KI_YAW)
                                   + (flight_control->delta_error[PID_YAW] * KD_YAW);
    flight_control->pid[PID_PITCH] = flight_control->error[PID_PITCH] * KP_PITCH + flight_control->error_sum[PID_PITCH] * KI_PITCH
                                    + flight_control->delta_error[PID_PITCH] * KD_PITCH;
    flight_control->pid[PID_ROLL] = (flight_control->error[PID_ROLL] * KP_ROLL) + (flight_control->error_sum[PID_ROLL] * KI_ROLL)
                                   + (flight_control->delta_error[PID_ROLL] * KD_ROLL);


    flight_control->output[0] = (uint32_t)(RC.diffCapture[RC_THROTTLE]/10*10 + flight_control->pid[PID_ROLL] - flight_control->pid[PID_PITCH] + flight_control->pid[PID_YAW]);
    flight_control->output[1] = (uint32_t)(RC.diffCapture[RC_THROTTLE]/10*10 - flight_control->pid[PID_ROLL] - flight_control->pid[PID_PITCH] - flight_control->pid[PID_YAW]);
    flight_control->output[2] = (uint32_t)(RC.diffCapture[RC_THROTTLE]/10*10 + flight_control->pid[PID_ROLL] + flight_control->pid[PID_PITCH] - flight_control->pid[PID_YAW]);
    flight_control->output[3] = (uint32_t)(RC.diffCapture[RC_THROTTLE]/10*10 - flight_control->pid[PID_ROLL] + flight_control->pid[PID_PITCH] + flight_control->pid[PID_YAW]);

    UL_PWM_SetUs(&brushless[0], flight_control->output[0]);
    UL_PWM_SetUs(&brushless[1], flight_control->output[1]);
    UL_PWM_SetUs(&brushless[2], flight_control->output[2]);
    UL_PWM_SetUs(&brushless[3], flight_control->output[3]);
}

//! LAND PID
void UL_flight_control_land_pid_controller(UL_flight_control_typedef* flight_control){

    uint32_t dir = (RC.diffCapture[RC_ROLL]/10-150)*2;
    uint32_t throttle = RC.diffCapture[RC_THROTTLE]/10*10;
    if (throttle > 1500)
        throttle = 1500;
    if (throttle > 1050) {
        UL_PWM_SetUs(&brushless[0], (uint32_t) (throttle + dir));
        UL_PWM_SetUs(&brushless[1], (uint32_t) (throttle - dir));
        UL_PWM_SetUs(&brushless[2], (uint32_t) (throttle + dir));
        UL_PWM_SetUs(&brushless[3], (uint32_t) (throttle - dir));
    } else {
        UL_PWM_SetUs(&brushless[0], 0);
        UL_PWM_SetUs(&brushless[1], 0);
        UL_PWM_SetUs(&brushless[2], 0);
        UL_PWM_SetUs(&brushless[3], 0);
    }

}

void UL_flight_control_print(UL_flight_control_typedef* flight_control){

    UL_TFT_ST7735_WriteString(&Tft, 25, 2, "Y", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteString(&Tft, 50, 2, "P", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteString(&Tft, 75, 2, "R", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteString(&Tft, 100, 2, "T", FONT_7x10, ST7735_WHITE, ST7735_BLACK);

    UL_TFT_ST7735_WriteString(&Tft, 1, 20, "RC", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 15, 20, RC.diffCapture[RC_YAW]/10, FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 44, 20, RC.diffCapture[RC_PITCH]/10, FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 72, 20, RC.diffCapture[RC_ROLL]/10, FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 100, 20, RC.diffCapture[RC_THROTTLE]/10, FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    //UL_TFT_ST7735_WriteNumber(&Tft, 100, 100, RC.diffCapture[RC_MODE]/10, FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    //UL_TFT_ST7735_WriteNumber(&Tft, 100, 120, RC.diffCapture[RC_ONOFF]/10, FONT_7x10, ST7735_WHITE, ST7735_BLACK);

    UL_TFT_ST7735_WriteString(&Tft, 1, 40, "IMU", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 25, 40, (uint32_t)abs(IMU.yaw), FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 50, 40, (uint32_t)abs(IMU.pitch), FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 75, 40, (uint32_t)abs(IMU.roll), FONT_7x10, ST7735_WHITE, ST7735_BLACK);

    UL_TFT_ST7735_WriteString(&Tft, 1, 60, "TAR", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 25, 60, (uint32_t)abs(flight_control->target[PID_YAW]), FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 50, 60, (uint32_t)abs(flight_control->target[PID_PITCH]), FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 75, 60, (uint32_t)abs(flight_control->target[PID_ROLL]), FONT_7x10, ST7735_WHITE, ST7735_BLACK);

    UL_TFT_ST7735_WriteString(&Tft, 1, 80, "ESC", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 25, 80, flight_control->output[0]/10, FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 50, 80, flight_control->output[1]/10, FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 75, 80, flight_control->output[2]/10, FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 100, 80, flight_control->output[3]/10, FONT_7x10, ST7735_WHITE, ST7735_BLACK);

}