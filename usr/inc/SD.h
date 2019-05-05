//
// Created by HUI Ka Yiu on 2019-05-03.
//

#ifndef PLANE_SD_H
#define PLANE_SD_H

#include "main.h"

extern SPI_HandleTypeDef hspi6;

// call before initializing any SPI devices
void UL_SD_Unselect();

// all procedures return 0 on success, < 0 on failure

int UL_SD_Init();
int UL_SD_GetBlocksNumber(uint32_t* num);
int UL_SD_ReadSingleBlock(uint32_t blockNum, uint8_t* buff); // sizeof(buff) == 512!
int UL_SD_WriteSingleBlock(uint32_t blockNum, const uint8_t* buff); // sizeof(buff) == 512!

// Read Multiple Blocks
int UL_SD_ReadBegin(uint32_t blockNum);
int UL_SD_ReadData(uint8_t* buff); // sizeof(buff) == 512!
int UL_SD_ReadEnd();

// Write Multiple Blocks
int UL_SD_WriteBegin(uint32_t blockNum);
int UL_SD_WriteData(const uint8_t* buff); // sizeof(buff) == 512!
int UL_SD_WriteEnd();

// TODO: read lock flag? CMD13, SEND_STATUS

#endif //PLANE_SD_H
