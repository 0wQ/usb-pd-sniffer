#include "usb_pd_monitor.h"

#include "ch32x035_usbpd.h"
#include "debug.h"
#include "usb_cdc_print.h"
#include "usb_pd_cc.h"
#include "usb_pd_message.h"

/* PD RX Buuffer */
__attribute__((aligned(4))) static uint8_t usb_pd_rx_buffer[PD_MSG_MAX_LEN];

/* CC 连接状态 */
static cc_state_t cc_state = {0};

/**
 * @brief  配置为接收模式
 */
static void set_rx_mode(void) {
    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= ~PD_ALL_CLR;
    USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | PD_DMA_EN;
    USBPD->DMA = (uint32_t)(uint8_t *)usb_pd_rx_buffer;
    USBPD->CONTROL &= ~PD_TX_EN;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;
    USBPD->CONTROL |= BMC_START;
    NVIC_EnableIRQ(USBPD_IRQn);
}

/**
 * @brief  初始化 USB PD 监听
 */
void usb_pd_monitor_init(void) {
    // 使能时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBPD, ENABLE);

    // 初始化 CC 引脚
    usb_pd_cc_init();

    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET;

    // 配置接收模式
    set_rx_mode();
}

/**
 * @brief  USB PD 监听处理函数
 */
void usb_pd_monitor_process(void) {
    // 从 buffer 读取并处理 PD 消息
    pd_msg_buffer_t *msg_buffer = get_message_buffer();
    while (msg_buffer->read_idx != msg_buffer->write_idx) {                     // 判断是否有新消息
        print_message(&msg_buffer->msgs[msg_buffer->read_idx]);                 // 打印消息
        msg_buffer->read_idx = (msg_buffer->read_idx + 1) % PD_MSG_BUFFER_SIZE; // 更新读指针
    }

    // 检测 CC 连接状态
    usb_pd_cc_check_connection(&cc_state);
}

/**
 * @brief  USB PD 中断处理函数
 */
void USBPD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USBPD_IRQHandler(void) {
    uint8_t status = USBPD->STATUS;
    uint16_t byte_cnt = USBPD->BMC_BYTE_CNT;

    if (status & IF_RX_ACT) {
        USBPD->STATUS |= IF_RX_ACT;

        if ((status & MASK_PD_STAT) && byte_cnt >= 6) {
            save_message(status, usb_pd_rx_buffer, byte_cnt);
        }
    }

    if (status & IF_RX_RESET) {
        USBPD->STATUS |= IF_RX_RESET;
        // usb_pd_cc_detach(&cc_state);
        // 将 RX_RESET 事件保存到消息缓冲区
        save_message(status, NULL, 0);
    }

    // if (status & BUF_ERR) {
    //     USBPD->STATUS |= BUF_ERR;
    //     cdc_acm_printf("\nBUF_ERR\n");
    // }
}
