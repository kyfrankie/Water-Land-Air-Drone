//
// Created by HUI Ka Yiu on 2019-03-08.
//

#include "flight_control.h"
#include "stdlib.h"

void UL_flight_control_stop_motor(){
    UL_PWM_SetUs(&brushless[0], 1000);
    UL_PWM_SetUs(&brushless[1], 1000);
    UL_PWM_SetUs(&brushless[2], 1000);
    UL_PWM_SetUs(&brushless[3], 1000);
}

void UL_flight_control_init(UL_flight_control_typedef* flightControl){
    for (int i = 0; i < 4; i++) {
        flightControl->error[i] = 0;
        flightControl->delta_error[i] = 0;
        flightControl->error_sum[i] = 0;
        flightControl->pid[i] = 0;
        flightControl->previous_error[i] = 0;
        flightControl->output[i] = 0;
    }

    /*
    for (int i = 0; i < 400; i++){
        flightControl->temp_imu[i] =0;
        flightControl->temp_err[i] = 0;
        flightControl->temp_delta[i] = 0;
        flightControl->temp_sum[i] = 0;
        flightControl->temp_out[i] = 0;
    }
    flightControl->index = 0;
    */

    flightControl->displacement_x = 0;
    flightControl->displacement_y = 0;
    flightControl->displacement_z = 0;
    flightControl->speed_x = 0;
    flightControl->speed_y = 0;
    flightControl->speed_z = 0;

    flightControl->target[PID_YAW] = IMU.yaw;
    UL_IMU_CAL(&IMU);

    flightControl->ready = 1;
}

void UL_flight_control(UL_flight_control_typedef* flight_control){
    if (RC.diffCapture[RC_MODE] < 1050){
        if (RC.diffCapture[RC_THROTTLE] < 1050 && flight_control->currentMode != MODE_LAND_F) {
            flight_control->currentMode = MODE_LAND_F;
            UL_PWM_SetUs(&servo[0], 1000);
            UL_PWM_SetUs(&servo[1], 2000);
            UL_PWM_SetUs(&servo[2], 1000);
            UL_PWM_SetUs(&servo[3], 2000);
        }
        if (RC.diffCapture[RC_THROTTLE] > 1050)
            UL_flight_control_land_pid_controller(flight_control);
        else UL_flight_control_stop_motor();
    }
    else if (RC.diffCapture[RC_MODE] > 1450 && RC.diffCapture[RC_MODE] < 1550) {
        if (RC.diffCapture[RC_THROTTLE] < 1050 && flight_control->currentMode != MODE_AIR){
            flight_control->currentMode = MODE_AIR;
            UL_flight_control_init(flight_control);
            UL_PWM_SetUs(&servo[0], 1550);
            UL_PWM_SetUs(&servo[1], 1500);
            UL_PWM_SetUs(&servo[2], 1550);
            UL_PWM_SetUs(&servo[3], 1500);
        }
        if (RC.diffCapture[RC_THROTTLE] > 1050)
            UL_flight_control_air_pid_controller(flight_control);
        else UL_flight_control_stop_motor();
    }
    else if (RC.diffCapture[RC_MODE] > 1950){
        if (RC.diffCapture[RC_THROTTLE] < 1050 && flight_control->currentMode != MODE_LAND_B){
            flight_control->currentMode = MODE_LAND_B;
            UL_PWM_SetUs(&servo[0], 2000);
            UL_PWM_SetUs(&servo[1], 1000);
            UL_PWM_SetUs(&servo[2], 2000);
            UL_PWM_SetUs(&servo[3], 1000);
        }
        if (RC.diffCapture[RC_THROTTLE] > 1050)
            UL_flight_control_land_pid_controller(flight_control);
        else UL_flight_control_stop_motor();
    }
}

//! AIR PID
void UL_flight_control_air_pid_controller(UL_flight_control_typedef* flight_control){

    uint32_t throttle = RC.diffCapture[RC_THROTTLE]/10*10;
    if (throttle < 1300)
        throttle = (throttle - 1000) * 2 + 1000;
    else if (throttle < 1900 && throttle > 1300)
        throttle = (throttle - 1300) / 2 + 1600;
    if (throttle > 1900)
        throttle = 1900;

    if (RC.diffCapture[RC_YAW]/10-150 > 20 || RC.diffCapture[RC_YAW]/10-150 < -20 ) {
        double temp = flight_control->target[PID_YAW] + (RC.diffCapture[RC_YAW] / 10 - 150)/1000.0;
        if (temp > 360.0) {
            flight_control->target[PID_YAW] = temp - 360.0;
        }
        else if (temp < 0.0) {
            flight_control->target[PID_YAW] = temp + 360.0;
        }
        else flight_control->target[PID_YAW] = temp;
    }

    flight_control->target[PID_PITCH] = (double)(RC.diffCapture[RC_PITCH]/10-150)/50.0 * -10.0;  //! limited to 10 degree
    flight_control->target[PID_ROLL] = (double)(RC.diffCapture[RC_ROLL]/10-150)/50.0 * 10.0;

    flight_control->error[PID_YAW] = flight_control->target[PID_YAW] - IMU.yaw;
    flight_control->error[PID_PITCH] = flight_control->target[PID_PITCH] - IMU.pitch;
    flight_control->error[PID_ROLL] = flight_control->target[PID_ROLL] - IMU.roll;

    flight_control->delta_error[PID_YAW] = flight_control->error[PID_YAW] - flight_control->previous_error[PID_YAW];
    flight_control->delta_error[PID_PITCH] = flight_control->error[PID_PITCH] - flight_control->previous_error[PID_PITCH];
    flight_control->delta_error[PID_ROLL] = flight_control->error[PID_ROLL] - flight_control->previous_error[PID_ROLL];

    if (RC.diffCapture[RC_THROTTLE] > 1500) {
        flight_control->error_sum[PID_YAW] += flight_control->error[PID_YAW];
        flight_control->error_sum[PID_PITCH] += flight_control->error[PID_PITCH];
        flight_control->error_sum[PID_ROLL] += flight_control->error[PID_YAW];
    }

    if (flight_control->error_sum[PID_YAW] > MAXI)
        flight_control->error_sum[PID_YAW] = MAXI;
    if (flight_control->error_sum[PID_YAW < -MAXI])
        flight_control->error_sum[PID_YAW] = -MAXI;
    if (flight_control->error_sum[PID_PITCH] > MAXI)
        flight_control->error_sum[PID_PITCH] = MAXI;
    if (flight_control->error_sum[PID_PITCH < -MAXI])
        flight_control->error_sum[PID_PITCH] = -MAXI;
    if (flight_control->error_sum[PID_ROLL] > MAXI)
        flight_control->error_sum[PID_ROLL] = MAXI;
    if (flight_control->error_sum[PID_ROLL < -MAXI])
        flight_control->error_sum[PID_ROLL] = -MAXI;

    flight_control->previous_error[PID_YAW] = flight_control->error[PID_YAW];
    flight_control->previous_error[PID_PITCH] = flight_control->error[PID_PITCH];
    flight_control->previous_error[PID_ROLL] = flight_control->error[PID_ROLL];

    flight_control->pid[PID_YAW] = (flight_control->error[PID_YAW] * KP_YAW) + (flight_control->error_sum[PID_YAW] * KI_YAW)
                                   + (flight_control->delta_error[PID_YAW] * KD_YAW);
    flight_control->pid[PID_PITCH] = flight_control->error[PID_PITCH] * KP_PITCH + flight_control->error_sum[PID_PITCH] * KI_PITCH
                                    + flight_control->delta_error[PID_PITCH] * KD_PITCH;
    flight_control->pid[PID_ROLL] = (flight_control->error[PID_ROLL] * KP_ROLL) + (flight_control->error_sum[PID_ROLL] * KI_ROLL)
                                   + (flight_control->delta_error[PID_ROLL] * KD_ROLL);

    //! Alltiude hold pid
    if (throttle > 1700 && throttle < 1800){
        flight_control->error[PID_ALL] = IMU.accel_z;
        flight_control->delta_error[PID_ALL] = flight_control->error[PID_ALL] - flight_control->previous_error[PID_ALL];
        flight_control->error_sum[PID_ALL] += flight_control->error[PID_ALL];
        flight_control->previous_error[PID_ALL] = flight_control->error[PID_ALL];
        flight_control->pid[PID_ALL] = (flight_control->error[PID_ALL] * KP_ALL) + (flight_control->delta_error[PID_ALL]*KD_ALL) + (flight_control->error_sum[PID_ALL]*KI_ALL);
    }

    flight_control->output[0] = (uint32_t)(throttle + flight_control->pid[PID_ROLL] - flight_control->pid[PID_PITCH] + flight_control->pid[PID_YAW] + flight_control->pid[PID_ALL])+ 60;
    flight_control->output[1] = (uint32_t)(throttle - flight_control->pid[PID_ROLL] - flight_control->pid[PID_PITCH] - flight_control->pid[PID_YAW] + flight_control->pid[PID_ALL]) + 60;
    flight_control->output[2] = (uint32_t)(throttle + flight_control->pid[PID_ROLL] + flight_control->pid[PID_PITCH] - flight_control->pid[PID_YAW] + flight_control->pid[PID_ALL]);
    flight_control->output[3] = (uint32_t)(throttle - flight_control->pid[PID_ROLL] + flight_control->pid[PID_PITCH] + flight_control->pid[PID_YAW] + flight_control->pid[PID_ALL]);

    if (flight_control->output[0] > 2000)
        flight_control->output[0] = 2000;
    if (flight_control->output[1] > 2000)
        flight_control->output[1] = 2000;
    if (flight_control->output[2] > 2000)
        flight_control->output[2] = 2000;
    if (flight_control->output[3] > 2000)
        flight_control->output[3] = 2000;

    /*
    if(flight_control->index < 200 && RC.diffCapture[RC_THROTTLE] > 1500) {

        flight_control->temp_throttle[flight_control->index] = RC.diffCapture[RC_THROTTLE];
        flight_control->temp_imu[flight_control->index] = IMU.pitch;
        flight_control->temp_err[flight_control->index] = flight_control->error[PID_PITCH];
        flight_control->temp_delta[flight_control->index] = flight_control->delta_error[PID_PITCH];
        flight_control->temp_sum[flight_control->index] = flight_control->error_sum[PID_PITCH];
        flight_control->temp_pid[flight_control->index] = flight_control->pid[PID_PITCH];
        flight_control->temp_out[flight_control->index] = flight_control->output[PID_PITCH];

        flight_control->output0[flight_control->index] = flight_control->output[0];
        flight_control->output1[flight_control->index] = flight_control->output[1];
        flight_control->output2[flight_control->index] = flight_control->output[2];
        flight_control->output3[flight_control->index] = flight_control->output[3];
        flight_control->index++;
    }
     */

    UL_PWM_SetUs(&brushless[0], flight_control->output[0]);
    UL_PWM_SetUs(&brushless[1], flight_control->output[1]);
    UL_PWM_SetUs(&brushless[2], flight_control->output[2]);
    UL_PWM_SetUs(&brushless[3], flight_control->output[3]);
}

//! LAND PID
void UL_flight_control_land_pid_controller(UL_flight_control_typedef* flight_control){

    uint32_t dir = (RC.diffCapture[RC_ROLL]/10-150)*2;
    uint32_t throttle = RC.diffCapture[RC_THROTTLE]/10*10;
    if (throttle > 1450)
        throttle = 1450;
    if (throttle > 1050) {
        UL_PWM_SetUs(&brushless[0], (uint32_t) (throttle + dir));
        UL_PWM_SetUs(&brushless[1], (uint32_t) (throttle - dir));
        UL_PWM_SetUs(&brushless[2], (uint32_t) (throttle + dir));
        UL_PWM_SetUs(&brushless[3], (uint32_t) (throttle - dir));
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
    UL_TFT_ST7735_WriteNumber(&Tft, 25, 60, (uint32_t)flight_control->target[PID_YAW], FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 50, 60, (uint32_t)abs(flight_control->target[PID_PITCH]), FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 75, 60, (uint32_t)abs(flight_control->target[PID_ROLL]), FONT_7x10, ST7735_WHITE, ST7735_BLACK);

    UL_TFT_ST7735_WriteString(&Tft, 1, 80, "ESC", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 25, 80, flight_control->output[0]/10, FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 50, 80, flight_control->output[1]/10, FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 75, 80, flight_control->output[2]/10, FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 100, 80, flight_control->output[3]/10, FONT_7x10, ST7735_WHITE, ST7735_BLACK);

    UL_TFT_ST7735_WriteString(&Tft, 1, 100, "Alt", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 25, 100, (uint32_t)IMU.accel_z, FONT_7x10, ST7735_WHITE, ST7735_BLACK);

}