//
// Created by 刘嘉俊 on 25-6-10.
//

#ifndef CTRBOARD_H7_ALL_WS2812_H
#define CTRBOARD_H7_ALL_WS2812_H

#include "stm32h7xx_hal.h"  // 根据你的芯片型号调整

#define WS2812_LED_NUM     2   // 支持的灯珠数量
#define WS2812_BRIGHTNESS  50  // 默认亮度百分比（0~100）

#define COLOR_RED       255, 0, 0
#define COLOR_GREEN     0, 255, 0
#define COLOR_BLUE      0, 0, 255
#define COLOR_WHITE     255, 255, 255
#define COLOR_YELLOW    255, 255, 0
#define COLOR_CYAN      0, 255, 255
#define COLOR_MAGENTA   255, 0, 255
#define COLOR_ORANGE    255, 128, 0
#define COLOR_PURPLE    128, 0, 255
#define COLOR_PINK      255, 102, 178
#define COLOR_OFF       0, 0, 0

void WS2812_SetBrightness(uint8_t brightness_percent);
void WS2812_SetRGB(uint8_t r, uint8_t g, uint8_t b);
void WS2812_Show();

#endif //CTRBOARD_H7_ALL_WS2812_H
