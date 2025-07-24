#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ch32x035.h"
#include "debug.h"

// LED length
#ifndef LED_STRIP_LENGTH
#define LED_STRIP_LENGTH 1
#endif

// LED color order
#define LED_STRIP_RGB 0
#define LED_STRIP_GRB 1
#define LED_STRIP_BRG 2

#ifndef LED_STRIP_ORDER
#define LED_STRIP_ORDER LED_STRIP_GRB  // 默认使用 GRB 顺序
#endif

void led_strip_init(void);
void led_strip_clear(void);
void led_strip_refresh(void);
void led_strip_set_pixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);
void led_strip_set_pixel_with_refresh(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);
void led_strip_set_pixel_hsv(uint16_t index, uint16_t hue, uint8_t saturation, uint8_t value);
