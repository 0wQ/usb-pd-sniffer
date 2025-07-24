#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ch32x035_gpio.h"

// ADC 引脚配置
#define ADC_GPIO_PORT GPIOA
#define ADC_GPIO_CLK  RCC_APB2Periph_GPIOA
#define ADC_GPIO_PIN  GPIO_Pin_1
#define ADC_CHANNEL   ADC_Channel_1
// #define ADC_GPIO_PORT GPIOB
// #define ADC_GPIO_CLK  RCC_APB2Periph_GPIOB
// #define ADC_GPIO_PIN  GPIO_Pin_0
// #define ADC_CHANNEL   ADC_Channel_8

#define ADC_CHANNEL_COUNT 2   // 采样的通道数量
#define ADC_SAMPLE_COUNT  256 // 每个通道采样次数
#define ADC_BUFFER_SIZE   (ADC_CHANNEL_COUNT * ADC_SAMPLE_COUNT)

// 校准点结构体
typedef struct {
    uint16_t actual;   // 实际电压值 (mV)
    uint16_t measured; // ADC 测量值 (mV)
} CalibrationPoint;

// 校准点数据 (需按 measured 值从小到大排序，关闭校准后测量)
static const CalibrationPoint VBUS_CAL_POINTS[] = {
    {0, 510},
    {5007, 5372},
    {8975, 9199},
    {11961, 12070},
    {20030, 19907},
};

// 校准参数
static const uint16_t ADC_VREF_VOLTAGE = 3300; // ADC 参考电压 (mV)
static const uint32_t VBUS_DIV_R1 = 68000;     // VBUS 上分压电阻 (Ω)
static const uint32_t VBUS_DIV_R2 = 4700;      // VBUS 下分压电阻 (Ω)
static const bool VBUS_CAL_ENABLE = 1;         // 是否启用校准

// static const uint16_t ADC_VREF_VOLTAGE = 3322; // ADC 参考电压 (mV)
// static const uint32_t VBUS_DIV_R1 = 100000;    // VBUS 上分压电阻 (Ω)
// static const uint32_t VBUS_DIV_R2 = 10000;     // VBUS 下分压电阻 (Ω)
// static const bool VBUS_CAL_ENABLE = 0;         // 是否启用校准

static const size_t VBUS_CAL_POINTS_COUNT = sizeof(VBUS_CAL_POINTS) / sizeof(CalibrationPoint);
static const float VBUS_CONVERT_SCALE = ADC_VREF_VOLTAGE / 4095.0f * ((float)(VBUS_DIV_R1 + VBUS_DIV_R2) / (float)VBUS_DIV_R2);

void adc_init();
uint16_t adc_get_avg_raw(void);
uint16_t adc_raw_to_vbus_mv(uint16_t adc_raw);
uint16_t adc_get_vbus_mv(void);
uint16_t adc_get_vdd_mv(void);
