//
// Created by 刘嘉俊 on 25-6-10.
//

#include "ws2812.h"

#define WS2812_HIGH 0xF0  // 代表逻辑1
#define WS2812_LOW  0xC0  // 代表逻辑0

static SPI_HandleTypeDef *ws2812_spi;
static uint8_t led_buffer[WS2812_LED_NUM][3];  // GRB 存储
static uint8_t brightness = WS2812_BRIGHTNESS;

void WS2812_Init(SPI_HandleTypeDef *hspi) {
    ws2812_spi = hspi;
}

void WS2812_SetBrightness(uint8_t brightness_percent) {
    if (brightness_percent > 100) brightness_percent = 100;
    brightness = brightness_percent;
}

void WS2812_SetRGB(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= WS2812_LED_NUM) return;

    // 应用亮度缩放
    r = (r * brightness) / 100;
    g = (g * brightness) / 100;
    b = (b * brightness) / 100;

    led_buffer[index][0] = g;
    led_buffer[index][1] = r;
    led_buffer[index][2] = b;
}

void WS2812_Show(void) {
    uint8_t spi_data[WS2812_LED_NUM * 3 * 8];  // 每bit → 1字节
    uint16_t idx = 0;

    for (int i = 0; i < WS2812_LED_NUM; i++) {
        for (int j = 0; j < 3; j++) {
            uint8_t val = led_buffer[i][j];
            for (int k = 0; k < 8; k++) {
                if (val & (1 << (7 - k))) {
                    spi_data[idx++] = WS2812_HIGH;
                } else {
                    spi_data[idx++] = WS2812_LOW;
                }
            }
        }
    }

    HAL_SPI_Transmit(ws2812_spi, spi_data, sizeof(spi_data), HAL_MAX_DELAY);
    HAL_Delay(1);  // Reset > 50us
}
