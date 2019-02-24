//
// Created by HUI Ka Yiu on 2018-12-27.
// Adjusted from https://github.com/afiskon/stm32-st7735
//

#ifndef PLANE_TFT_H
#define PLANE_TFT_H

#include <stdbool.h>
#include "stm32h7xx.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_gpio.h"
#include "tft_defines.h"

typedef struct {
    GPIO_TypeDef* GPIO_RST_Port;
    uint16_t GPIO_RST_Pin;
    GPIO_TypeDef* GPIO_DC_Port;
    uint16_t GPIO_DC_Pin;
    GPIO_TypeDef* GPIO_CS_Port;
    uint16_t GPIO_CS_Pin;
    SPI_HandleTypeDef* hspi;
} UL_TFT_typedef;

void UL_TFT_ST7735_Init(UL_TFT_typedef *TFT, GPIO_TypeDef *GPIO_RST_Port, uint16_t GPIO_RST_Pin,
                        GPIO_TypeDef *GPIO_DC_Port, uint16_t GPIO_DC_Pin,
                        GPIO_TypeDef *GPIO_CS_Port, uint16_t GPIO_CS_Pin, SPI_HandleTypeDef *hspi);
void UL_TFT_ST7735_DrawPixel(UL_TFT_typedef *TFT, uint16_t x, uint16_t y, uint16_t color);
void UL_TFT_ST7735_WriteString(UL_TFT_typedef *TFT, uint16_t x, uint16_t y, const char *str, FontDef font,
                               uint16_t color, uint16_t bgcolor);
void UL_TFT_ST7735_WriteNumber(UL_TFT_typedef *TFT, uint16_t x, uint16_t y, const uint32_t num, FontDef font,
                               uint16_t color, uint16_t bgcolor);
void UL_TFT_ST7735_FillRectangle(UL_TFT_typedef *TFT, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void UL_TFT_ST7735_FillScreen(UL_TFT_typedef *TFT, uint16_t color);
void UL_TFT_ST7735_DrawImage(UL_TFT_typedef *TFT, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);
void UL_TFT_ST7735_InvertColors(UL_TFT_typedef *TFT, bool invert);




#endif //PLANE_TFT_H
