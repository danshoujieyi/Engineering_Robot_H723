//
// Created by 刘嘉俊 on 25-6-10.
//

#include "ws2812.h"

#define WS2812_HighLevel 0xF0  // 代表逻辑1
#define WS2812_LowLevel  0xC0  // 代表逻辑0

#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;
static uint8_t brightness = WS2812_BRIGHTNESS;

static uint8_t red, green, blue;

void WS2812_SetBrightness(uint8_t brightness_percent) {
    if (brightness_percent > 100) brightness_percent = 100;
    brightness = brightness_percent;
}

void WS2812_SetRGB(uint8_t r, uint8_t g, uint8_t b) {
    // 应用亮度缩放
    red = (r * brightness) / 100;
    green = (g * brightness) / 100;
    blue = (b * brightness) / 100;

}

void WS2812_Show()
{
    uint8_t txbuf[24];
    uint8_t res = 0;
    for (int i = 0; i < 8; i++)
    {
        txbuf[7-i]  = (((green>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[15-i] = (((red>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[23-i] = (((blue>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 0, 0xFFFF);
   // HAL_SPI_Transmit_DMA(&WS2812_SPI_UNIT, &res, 0);
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);
   // HAL_SPI_Transmit_DMA(&WS2812_SPI_UNIT, txbuf, 24);
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 1, 0xFFFF);
    //    HAL_SPI_Transmit_DMA(&WS2812_SPI_UNIT, &res, 1);
    }
}