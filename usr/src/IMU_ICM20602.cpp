//
// Created by HUI Ka Yiu on 2019-04-27.
//

#include "IMU_ICM20602.h"

void UL_IMU_Select(){
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

void UL_IMU_DeSelect(){
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}
void UL_IMU_ICM_Init(UL_IMU_ICM_typedef* IMU, SPI_HandleTypeDef* hspi){
    UL_IMU_Select();
    UL_IMU_ICM_Write_Reg(IMU, ICM20_PWR_MGMT_1, 0x80);      //reset
    HAL_Delay(100);
    UL_IMU_ICM_Write_Reg(IMU, ICM20_PWR_MGMT_1, 0x01);      //wake up, auto find clock source
    HAL_Delay(100);
    UL_IMU_ICM_Write_Reg(IMU, ICM20_PWR_MGMT_2, 0x3F);      //disable gyro and accel
    HAL_Delay(100);

    UL_IMU_ICM_Write_Reg(IMU, ICM20_I2C_IF, 0x40);          //disable I2C
    UL_IMU_ICM_Write_Reg(IMU, ICM20_ACCEL_INTEL_CTRL, 0x02);//avoid limit sensor output
    //UL_IMU_Write_Reg(IMU, 17, 0xC9);                  //bug fix?

    UL_IMU_ICM_Write_Reg(IMU, ICM20_INT_ENABLE, 0x00);      //disable interupt
    UL_IMU_ICM_Write_Reg(IMU, ICM20_FIFO_EN, 0x00);         //disable FIFO
    UL_IMU_ICM_Write_Reg(IMU, ICM20_USER_CTRL, 0x00);

    UL_IMU_ICM_Write_Reg(IMU, ICM20_GYRO_CONFIG, 0x19);     //2000dps gyro, 32khx
    HAL_Delay(50);
    UL_IMU_ICM_Write_Reg(IMU, ICM20_ACCEL_CONFIG, 0x18);    //16g accel
    HAL_Delay(50);
    UL_IMU_ICM_Write_Reg(IMU, ICM20_ACCEL_CONFIG2, 0x04);   //4khz

    UL_IMU_ICM_Write_Reg(IMU, ICM20_PWR_MGMT_2, 0x00);      //enable imu

    UL_IMU_DeSelect();
}

void UL_IMU_ICM_SetUp(UL_IMU_ICM_typedef* IMU){

}

void UL_IMU_ICM_Read(UL_IMU_ICM_typedef* IMU){
    UL_IMU_Select();
    uint8_t rx_gyro[6], rx_accel[6];
    rx_gyro[0] = UL_IMU_ICM_Read_Reg(IMU, ICM20_GYRO_XOUT_H);
    rx_gyro[1] = UL_IMU_ICM_Read_Reg(IMU, ICM20_GYRO_XOUT_L);
    rx_gyro[2] = UL_IMU_ICM_Read_Reg(IMU, ICM20_GYRO_YOUT_H);
    rx_gyro[3] = UL_IMU_ICM_Read_Reg(IMU, ICM20_GYRO_YOUT_L);
    rx_gyro[4] = UL_IMU_ICM_Read_Reg(IMU, ICM20_GYRO_ZOUT_H);
    rx_gyro[5] = UL_IMU_ICM_Read_Reg(IMU, ICM20_GYRO_ZOUT_L);
    rx_accel[0] = UL_IMU_ICM_Read_Reg(IMU, ICM20_ACCEL_XOUT_H);
    rx_accel[1] = UL_IMU_ICM_Read_Reg(IMU, ICM20_ACCEL_XOUT_L);
    rx_accel[2] = UL_IMU_ICM_Read_Reg(IMU, ICM20_ACCEL_YOUT_H);
    rx_accel[3] = UL_IMU_ICM_Read_Reg(IMU, ICM20_ACCEL_YOUT_L);
    rx_accel[4] = UL_IMU_ICM_Read_Reg(IMU, ICM20_ACCEL_ZOUT_H);
    rx_accel[5] = UL_IMU_ICM_Read_Reg(IMU, ICM20_ACCEL_ZOUT_L);

    int16_t gryo[3], accel[3];
    gryo[0] = ((int16_t)rx_gyro[0]<<8) + rx_gyro[1];
    gryo[1] = ((int16_t)rx_gyro[2]<<8) + rx_gyro[3];
    gryo[2] = ((int16_t)rx_gyro[4]<<8) + rx_gyro[5];
    accel[0] = ((int16_t)rx_accel[0]<<8) + rx_accel[1];
    accel[1] = ((int16_t)rx_accel[2]<<8) + rx_accel[3];
    accel[2] = ((int16_t)rx_accel[4]<<8) + rx_accel[5];


    IMU->raw_gyro[0] = (double)gryo[0] / 16.4;
    IMU->raw_gyro[1] = (double)gryo[1] / 16.4;
    IMU->raw_gyro[2] = (double)gryo[2] / 16.4;

    IMU->raw_accel[0] = (double)accel[0] / 2048.0;
    IMU->raw_accel[1] = (double)accel[1] / 2048.0;
    IMU->raw_accel[2] = (double)accel[2] / 2048.0;
}

void UL_IMU_ICM_Write_Reg(UL_IMU_ICM_typedef* IMU, uint8_t regAdress, uint8_t value){
    uint8_t tx = regAdress & 0x7F;
    HAL_SPI_Transmit(IMU->hspi, &regAdress, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(IMU->hspi, &value, 1, HAL_MAX_DELAY);
}

uint8_t UL_IMU_ICM_Read_Reg(UL_IMU_ICM_typedef* IMU, uint8_t regAdress){
    uint8_t ret, value;
    ret = regAdress | 0x80;
    HAL_SPI_Transmit(IMU->hspi, &ret, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(IMU->hspi, &value, 1, HAL_MAX_DELAY);
    return value;
}
