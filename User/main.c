#include "ch32x035.h"
#include "debug.h"
#include "led_strip.h"
#include "millis.h"
#include "usb_cdc_print.h"
#include "usb_pd_monitor.h"
#include "usb_pd_cc.h"
#include "usb_vbus_measure.h"

static void led_strip_rainbow_effect(void) {
    uint8_t brightness = 0;
    const uint8_t target_brightness = 0x0A;
    const uint16_t steps = 360;
    const float brightness_step = (float)target_brightness / steps;

    for (uint16_t hue = 0; hue < steps; hue++) {
        brightness = (uint8_t)(hue * brightness_step);
        if (brightness > target_brightness) {
            brightness = target_brightness;
        }
        led_strip_set_pixel_hsv(0, hue, 220, brightness);
        led_strip_refresh();
        Delay_Ms(3);
    }
}

int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();

    Delay_Init();
    // SDI_Printf_Enable();
    // printf("SystemClk:%d\n", SystemCoreClock);

    millis_init();
    adc_init();

    // LED Strip
    led_strip_init();
    led_strip_rainbow_effect();

    // USB PD Monitor
    usb_pd_monitor_init();

    // USB CDC
    cdc_acm_init(0, 0);
    while (!cdc_acm_is_configured()) {
        __NOP();
    }
    cdc_acm_prints("\n"); // 等待 ep_tx_busy_flag 串口打开后才会继续执行
    led_strip_set_pixel_with_refresh(0, 0x0A, 0x00, 0x00);
    Delay_Ms(500);
    cdc_acm_prints("\nUSB PD Sniffer - 开启 DTR 显示调试信息\n");

    // cc_en
    usb_pd_cc_en(true);

    while (1) {
        static uint32_t last_process_millis = 0;
        if (millis() - last_process_millis >= 10) {
            last_process_millis = millis();
            usb_pd_monitor_process();
        }

        // rd_en control
        // if (cdc_acm_get_dtr()) {
        //     usb_pd_cc_rd_en(true);
        // } else {
        //     usb_pd_cc_rd_en(false);
        // }
    }
}
