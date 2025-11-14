#include "usb_pd_monitor.h"

#include "ch32x035_usbpd.h"
#include "debug.h"
#include "usb_cdc_print.h"
#include "usb_pd_cc.h"
#include "usb_pd_message.h"
#include "usb_pd_snk.h"
#include "usb_pd_auto.h"

/* PD RX Buuffer */
__attribute__((aligned(4))) static uint8_t usb_pd_rx_buffer[PD_MSG_MAX_LEN];
/* GoodCRC logging: store header until TX_END for proper ordering */
static volatile uint8_t s_ack_hdr[2] = {0};
static volatile uint8_t s_ack_pending = 0;

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
    /* flush any deferred SNK prints in non-ISR context */
    usb_pd_snk_poll();
    usb_pd_auto_poll();

    // 从 buffer 读取并处理 PD 消息
    pd_msg_buffer_t *msg_buffer = get_message_buffer();
    while (msg_buffer->read_idx != msg_buffer->write_idx) {                     // 判断是否有新消息
        pd_msg_t *m = &msg_buffer->msgs[msg_buffer->read_idx];
        print_message(m);                                                       // 打印消息
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
            /* Auto GoodCRC when SNK mode is active and RX is a non-GoodCRC SOP0 frame */
            if (usb_pd_snk_is_active() && ((status & MASK_PD_STAT) == PD_RX_SOP0)) {
                /* Header at rx buffer */
                uint8_t *rx = usb_pd_rx_buffer; /* same DMA buffer registered */
                bool is_goodcrc = ((rx[0] & 0x1F) == CTRL_GOODCRC) && (byte_cnt == 6);
                if (!is_goodcrc) {
                    /* Respond GoodCRC ~30us later */
                    Delay_Us(30);
                    static uint8_t ack[2];
                    ack[0] = (uint8_t)(0x01 | usb_pd_snk_get_spec_flag()); /* GoodCRC with selected SpecRev */
                    ack[1] = (rx[1] & 0x0E);                         /* echo MsgID, PRRole forced to 0 (SNK) */

                    /* Drive selected CC */
                    if ((USBPD->CONFIG & CC_SEL) == CC_SEL) {
                        USBPD->PORT_CC2 |= CC_LVE;
                    } else {
                        USBPD->PORT_CC1 |= CC_LVE;
                    }

                    USBPD->CONFIG |= IE_TX_END;
                    USBPD->BMC_CLK_CNT = UPD_TMR_TX_48M;
                    USBPD->DMA = (uint32_t)(uint8_t *)ack;
                    USBPD->TX_SEL = UPD_SOP0;
                    USBPD->BMC_TX_SZ = 2;
                    USBPD->CONTROL |= PD_TX_EN;
                    USBPD->STATUS &= BMC_AUX_INVALID;
                    USBPD->CONTROL |= BMC_START;

                    /* Defer logging GoodCRC until TX_END so it appears after the triggering RX */
                    s_ack_hdr[0] = ack[0];
                    s_ack_hdr[1] = ack[1];
                    s_ack_pending = 1;

                    /* After scheduling GoodCRC, evaluate auto-replies and queue them */
                    usb_pd_auto_on_rx(status, rx, byte_cnt);
                }
            }
            save_message(status, usb_pd_rx_buffer, byte_cnt);
        }
    }

    if (status & IF_RX_RESET) {
        USBPD->STATUS |= IF_RX_RESET;
        // usb_pd_cc_detach(&cc_state);
        // 将 RX_RESET 事件保存到消息缓冲区
        save_message(status, NULL, 0);
    }

    if (status & IF_TX_END) {
        /* TX complete (e.g., GoodCRC or a data/control frame) */
        USBPD->STATUS |= IF_TX_END;

        /* If a GoodCRC was just sent, log it now to preserve ordering */
        if (s_ack_pending) {
            uint8_t tmp[2] = { (uint8_t)s_ack_hdr[0], (uint8_t)s_ack_hdr[1] };
            save_message(PD_RX_SOP0, tmp, 2);
            s_ack_pending = 0;
        }

        /* If a pending auto-reply exists, start it immediately; otherwise, release CC drive and return to RX */
        if (!usb_pd_snk_on_tx_end_handle_pending()) {
            USBPD->PORT_CC1 &= ~CC_LVE;
            USBPD->PORT_CC2 &= ~CC_LVE;
            USBPD->CONTROL &= ~PD_TX_EN;
            USBPD->DMA = (uint32_t)(uint8_t *)usb_pd_rx_buffer;
            USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;
            USBPD->CONTROL |= BMC_START;
        }
    }

    // if (status & BUF_ERR) {
    //     USBPD->STATUS |= BUF_ERR;
    //     cdc_acm_printf("\nBUF_ERR\n");
    // }
}
