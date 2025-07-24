#include "usb_pd_cc.h"

#include "ch32x035_usbpd.h"
#include "debug.h"
#include "led_strip.h"
#include "millis.h"
#include "usb_cdc_print.h"
#include "usb_vbus_measure.h"

#define USE_CC_CTRL
#define USE_CC_RD_CTRL

extern void reset_message_counter(void);

void usb_pd_cc_en(bool en) {
#ifdef USE_CC_CTRL
    static uint8_t inited = 0;

    if (!inited) {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        inited = 1;
    }

    GPIO_WriteBit(GPIOB, GPIO_Pin_0, en ? Bit_SET : Bit_RESET);
#endif
}

void usb_pd_cc_rd_en(bool en) {
#ifdef USE_CC_RD_CTRL
    static uint8_t inited = 0;

    if (!inited) {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        inited = 1;
    }

    GPIO_WriteBit(GPIOB, GPIO_Pin_12, en ? Bit_SET : Bit_RESET);
#endif
}

/**
 * @brief  获取 CC 电压
 * @param  cc_num: 1:CC1, 2:CC2
 * @param  is_connected: 是否已连接
 * @return uint16_t CC voltage
 */
static uint16_t usb_pd_phy_cc_get_voltage(uint8_t cc_num, bool is_connected) {
    // 从低到高依次检测电压
    const uint16_t cc_cmp_list[] = {CC_CMP_22, CC_CMP_45, CC_CMP_55, CC_CMP_66};
    const uint8_t cc_vol_list[] = {22, 45, 55, 66, 95, 123};

    register uint32_t PIN_CC;
    register volatile uint16_t *UBSPD_PORT_CC;

    switch (cc_num) {
    case 1:
        PIN_CC = PIN_CC1;
        UBSPD_PORT_CC = &USBPD->PORT_CC1;
        break;
    case 2:
        PIN_CC = PIN_CC2;
        UBSPD_PORT_CC = &USBPD->PORT_CC2;
        break;
    default:
        return 0;
    }

    // 读取 GPIO 引脚状态
    if ((GPIOC->INDR & PIN_CC) != (uint32_t)Bit_RESET) {
        return 220;
    }

    // 在已连接时，只比较 CC_CMP_66
    if (is_connected) {
        if (*UBSPD_PORT_CC & PA_CC_AI) {
            return 66;
        }
    }

    // 未连接时，从低到高依次检测电压
    if (!is_connected) {
        uint16_t cc_vol = 0;
        for (uint8_t i = 0; i < sizeof(cc_cmp_list) / sizeof(cc_cmp_list[0]); i++) {
            *UBSPD_PORT_CC &= ~(CC_CMP_Mask | PA_CC_AI);
            *UBSPD_PORT_CC |= cc_cmp_list[i];
            if (*UBSPD_PORT_CC & PA_CC_AI) {
                cc_vol = cc_vol_list[i];
            } else {
                break;
            }
        }
        *UBSPD_PORT_CC = (*UBSPD_PORT_CC & ~(CC_CMP_Mask | PA_CC_AI)) | CC_CMP_66;
        return cc_vol;
    }

    return 0;
}

/**
 * @brief  初始化 CC 引脚
 */
void usb_pd_cc_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    AFIO->CTLR |= USBPD_IN_HVT | USBPD_PHY_V33;
    USBPD->PORT_CC1 &= ~CC_LVE;
    USBPD->PORT_CC2 &= ~CC_LVE;
}

/**
 * @brief  检测 CC 连接状态（懒得改了凑合用）
 * @param  cc_state: CC 状态
 */
void usb_pd_cc_check_connection(cc_state_t *cc_state) {
    uint16_t cc1_volt, cc2_volt;
    bool cond;
    uint16_t vbus_volt = adc_get_vbus_mv();

    // 获取 CC1 和 CC2 的电压
    cc1_volt = usb_pd_phy_cc_get_voltage(1, cc_state->cc_connection);
    cc2_volt = usb_pd_phy_cc_get_voltage(2, cc_state->cc_connection);

    // 开启 DTR 时打印调试信息
    if (cdc_acm_get_dtr()) {
        static uint32_t last_print_time = 0;
        if (millis() - last_print_time > 100) {
            last_print_time = millis();
            cdc_acm_printf("COND:%u, CC1:%03umV, CC2:%03umV, VBUS:%05umV, VDD:%dmV\n", cc_state->cc_connection, cc1_volt, cc2_volt, vbus_volt, adc_get_vdd_mv());
        }
    }

    if (vbus_volt < 1000) {
        if (cc_state->cc_connection) {
            cdc_acm_printf("%ums Detach:CC%u, CC1:%03umV, CC2:%03umV, VBUS:%05umV\n", millis(), cc_state->cc_connection, cc1_volt, cc2_volt, vbus_volt);
            usb_pd_cc_detach(cc_state);
        }
        return;
    }

    // 当 vbus 有电压之后，才开启 cc_en
    usb_pd_cc_en(vbus_volt > 1000);

    if (cc_state->cc_connection) {
        // 已连接状态下的检测
        cond = (cc1_volt >= 45 && cc1_volt <= 123) || (cc2_volt >= 45 && cc2_volt <= 123);
        // 0 220
        if ((cc1_volt == 0 && cc2_volt == 220) || (cc1_volt == 220 && cc2_volt == 0)) {
            cond = false;
        }
        // 0 66
        if ((cc1_volt == 0 && cc2_volt == 66) || (cc1_volt == 66 && cc2_volt == 0)) {
            cond = true;
        }
        cc_state->cc_connection_count = cond ? 0 : cc_state->cc_connection_count + 1;

        // 如果连续检测到未连接，则认为连接断开
        if (!cond && cc_state->cc_connection_count >= 10) {
            cdc_acm_printf("%ums Detach:CC%u, CC1:%03umV, CC2:%03umV, VBUS:%05umV\n", millis(), cc_state->cc_connection, cc1_volt, cc2_volt, vbus_volt);
            usb_pd_cc_detach(cc_state);
        }
    } else {
        // 未连接状态下的检测
        cond = (cc1_volt >= 45 && cc1_volt <= 123 && (cc2_volt >= 220 || cc2_volt <= 22)) ||
               (cc2_volt >= 45 && cc2_volt <= 123 && (cc1_volt >= 220 || cc1_volt <= 22));
        // 220 220
        if (cc1_volt == 220 || cc2_volt == 220) {
            cond = false;
        }
        // 123 220
        if ((cc1_volt == 123 && cc2_volt == 220) || (cc1_volt == 220 && cc2_volt == 123)) {
            cond = true;
        }
        // 95 220
        if ((cc1_volt == 95 && cc2_volt == 220) || (cc1_volt == 220 && cc2_volt == 95)) {
            cond = true;
        }
        // 66 220
        if ((cc1_volt == 66 && cc2_volt == 220) || (cc1_volt == 220 && cc2_volt == 66)) {
            cond = true;
        }
        // 55 220
        if ((cc1_volt == 55 && cc2_volt == 220) || (cc1_volt == 220 && cc2_volt == 55)) {
            cond = true;
        }
        // 0 220
        if ((cc1_volt == 0 && cc2_volt == 220) || (cc1_volt == 220 && cc2_volt == 0)) {
            cond = false;
        }
        // 0 66
        if ((cc1_volt == 0 && cc2_volt == 66) || (cc1_volt == 66 && cc2_volt == 0)) {
            cond = true;
        }
        cc_state->cc_connection_count = cond ? cc_state->cc_connection_count + 1 : 0;

        // 如果连续检测到连接，则认为连接成功
        if (cond && cc_state->cc_connection_count >= 3) {
            if ((cc1_volt == 123 || cc1_volt == 95 || cc1_volt == 66 || cc1_volt == 55) && cc2_volt == 220) {
                cc_state->cc_connection = 1;
            } else if ((cc2_volt == 123 || cc2_volt == 95 || cc2_volt == 66 || cc2_volt == 55) && cc1_volt == 220) {
                cc_state->cc_connection = 2;
            } else {
                cc_state->cc_connection = cc1_volt > cc2_volt ? 1 : 2;
            }

            cc_state->cc_connection_count = 0;

            if (cc_state->cc_connection == 1) {
                USBPD->CONFIG &= ~CC_SEL;
                led_strip_set_pixel_with_refresh(0, 0x00, 0x00, 0x0A); // RGB BLUE
                cdc_acm_printf("%ums Attach:CC1, CC1:%03umV, CC2:%03umV, VBUS:%05umV\n", millis(), cc1_volt, cc2_volt, vbus_volt);
            }
            if (cc_state->cc_connection == 2) {
                USBPD->CONFIG |= CC_SEL;
                led_strip_set_pixel_with_refresh(0, 0x00, 0x0A, 0x00); // RGB GREEN
                cdc_acm_printf("%ums Attach:CC2, CC1:%03umV, CC2:%03umV, VBUS:%05umV\n", millis(), cc1_volt, cc2_volt, vbus_volt);
            }
        }
    }
}

void usb_pd_cc_detach(cc_state_t *cc_state) {
    cc_state->cc_connection = 0;
    cc_state->cc_connection_count = 0;
    reset_message_counter();
    led_strip_set_pixel_with_refresh(0, 0x0A, 0x00, 0x00); // RGB RED
}
