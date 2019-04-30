//
// Created by HUI Ka Yiu on 2019-04-27.
//

#ifndef PLANE_IMU_ICM20602_H
#define PLANE_IMU_ICM20602_H

#include "main.h"
#include "stdlib.h"

typedef struct {
    SPI_HandleTypeDef* hspi;
    double AccelResolution, GryroResolution;
    double raw_gyro[3], raw_accel[3];
    double roll, pitch, yaw, accel_x, accel_y, accel_z;
}UL_IMU_ICM_typedef;

void UL_IMU_ICM_Init(UL_IMU_ICM_typedef* IMU, SPI_HandleTypeDef* hspi);
void UL_IMU_ICM_SetUp(UL_IMU_ICM_typedef* IMU);
void UL_IMU_ICM_Read(UL_IMU_ICM_typedef* IMU);
void UL_IMU_ICM_Write_Reg(UL_IMU_ICM_typedef* IMU, uint8_t regAdress, uint8_t value);
uint8_t UL_IMU_ICM_Read_Reg(UL_IMU_ICM_typedef* IMU, uint8_t regAdress);


//========ICM20602 Register Adress========================
/********************************************
*Reset value of all register is 0，except
*Register 26  CONFIG				= 0x80
*Register 107 Power Management 1 	= 0x41
*Register 117 WHO_AM_I 				= 0x12
******************************************/

//Gyro temp compensate
#define	ICM20_XG_OFFS_TC_H				0x04
#define	ICM20_XG_OFFS_TC_L				0x05
#define	ICM20_YG_OFFS_TC_H				0x07
#define	ICM20_YG_OFFS_TC_L				0x08
#define	ICM20_ZG_OFFS_TC_H				0x0A
#define	ICM20_ZG_OFFS_TC_L				0x0B

//Accelerometer self check
#define	ICM20_SELF_TEST_X_ACCEL			0x0D
#define	ICM20_SELF_TEST_Y_ACCEL			0x0E
#define	ICM20_SELF_TEST_Z_ACCEL			0x0F

//Gryo offset
#define	ICM20_XG_OFFS_USRH				0x13
#define	ICM20_XG_OFFS_USRL				0x14
#define	ICM20_YG_OFFS_USRH				0x15
#define	ICM20_YG_OFFS_USRL				0x16
#define	ICM20_ZG_OFFS_USRH				0x17
#define	ICM20_ZG_OFFS_USRL				0x18

#define	ICM20_SMPLRT_DIV				0x19    //sample rate divider
#define	ICM20_CONFIG					0x1A
#define	ICM20_GYRO_CONFIG				0x1B
#define	ICM20_ACCEL_CONFIG				0x1C
#define	ICM20_ACCEL_CONFIG2				0x1D
#define	ICM20_LP_MODE_CFG				0x1E

//Wake-on motion threshold
#define	ICM20_ACCEL_WOM_X_THR			0x20
#define	ICM20_ACCEL_WOM_Y_THR			0x21
#define	ICM20_ACCEL_WOM_Z_THR			0x22


#define	ICM20_FIFO_EN					0x23
#define	ICM20_FSYNC_INT					0x36
#define	ICM20_INT_PIN_CFG				0x37
#define	ICM20_INT_ENABLE				0x38
#define	ICM20_FIFO_WM_INT_STATUS		0x39
#define	ICM20_INT_STATUS				0x3A

//Accelerometer output
#define	ICM20_ACCEL_XOUT_H				0x3B
#define	ICM20_ACCEL_XOUT_L				0x3C
#define	ICM20_ACCEL_YOUT_H				0x3D
#define	ICM20_ACCEL_YOUT_L				0x3E
#define	ICM20_ACCEL_ZOUT_H				0x3F
#define	ICM20_ACCEL_ZOUT_L				0x40

//Temp_out
#define	ICM20_TEMP_OUT_H				0x41
#define	ICM20_TEMP_OUT_L				0x42

//Gyro_out
#define	ICM20_GYRO_XOUT_H				0x43
#define	ICM20_GYRO_XOUT_L				0x44
#define	ICM20_GYRO_YOUT_H				0x45
#define	ICM20_GYRO_YOUT_L				0x46
#define	ICM20_GYRO_ZOUT_H				0x47
#define	ICM20_GYRO_ZOUT_L				0x48

//Gyro self test
#define	ICM20_SELF_TEST_X_GYRO			0x50
#define	ICM20_SELF_TEST_Y_GYRO			0x51
#define	ICM20_SELF_TEST_Z_GYRO			0x52

#define	ICM20_FIFO_WM_TH1				0x60
#define	ICM20_FIFO_WM_TH2				0x61
#define	ICM20_SIGNAL_PATH_RESET			0x68
#define	ICM20_ACCEL_INTEL_CTRL 			0x69
#define	ICM20_USER_CTRL					0x6A

//Power management
#define	ICM20_PWR_MGMT_1				0x6B
#define	ICM20_PWR_MGMT_2				0x6C

#define	ICM20_I2C_IF					0x70
#define	ICM20_FIFO_COUNTH				0x72
#define	ICM20_FIFO_COUNTL				0x73
#define	ICM20_FIFO_R_W					0x74

#define	ICM20_WHO_AM_I 					0x75

//Acceolerometer offset
#define	ICM20_XA_OFFSET_H				0x77
#define	ICM20_XA_OFFSET_L				0x78
#define	ICM20_YA_OFFSET_H				0x7A
#define	ICM20_YA_OFFSET_L				0x7B
#define	ICM20_ZA_OFFSET_H				0x7D

#endif //PLANE_IMU_ICM20602_H