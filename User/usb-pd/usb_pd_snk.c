#include "usb_pd_snk.h"

#include "ch32x035_usbpd.h"
#include "debug.h"
#include "usb_cdc_print.h"
#include "usb_pd_cc.h"
#include "usb_pd_message.h"
#include "usb_pd_auto.h"

static volatile bool s_snk_active = false;
static uint8_t s_tx_buf[34] __attribute__((aligned(4)));
static uint8_t s_tx_msg_id = 0; /* use bits [1..3], step by 2 */
static bool s_rdo_sent_once = false; /* avoid duplicate REQUEST on repeated SRC_CAP */
static volatile uint8_t s_deferred_msgs = 0; /* bit0: enter msg, bit1: exit msg, bit2: RDO sent msg */
static uint8_t s_pending_frame[34];
static uint8_t s_pending_len = 0;
static volatile uint8_t s_spec_rev = 2; /* 2 for PD2.0, 3 for PD3.0; default PD2.0 */

/* Select currently configured CC line */
static inline void pd_set_cc_drive_enable(bool en) {
    if ((USBPD->CONFIG & CC_SEL) == CC_SEL) {
        if (en) USBPD->PORT_CC2 |= CC_LVE; else USBPD->PORT_CC2 &= ~CC_LVE;
    } else {
        if (en) USBPD->PORT_CC1 |= CC_LVE; else USBPD->PORT_CC1 &= ~CC_LVE;
    }
}

/* Switch PHY to receive mode; keep current DMA pointer */
static inline void pd_switch_to_rx_mode(void) {
    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= ~PD_ALL_CLR;
    USBPD->CONTROL &= ~PD_TX_EN;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;
    USBPD->CONTROL |= BMC_START;
    NVIC_EnableIRQ(USBPD_IRQn);
}

/* Low-level PD TX (blocking option) */
static void pd_phy_send(bool wait_done, uint8_t *buf, uint8_t len, uint8_t sop_sel) {
    pd_set_cc_drive_enable(true);

    USBPD->BMC_CLK_CNT = UPD_TMR_TX_48M;
    USBPD->DMA = (uint32_t)(uint8_t *)buf;
    USBPD->TX_SEL = sop_sel;
    USBPD->BMC_TX_SZ = len;
    USBPD->CONTROL |= PD_TX_EN;
    USBPD->STATUS &= BMC_AUX_INVALID;
    USBPD->CONTROL |= BMC_START;

    if (wait_done) {
        while ((USBPD->STATUS & IF_TX_END) == 0) {
            __NOP();
        }
        USBPD->STATUS |= IF_TX_END;
        pd_set_cc_drive_enable(false);
        /* back to RX to receive GoodCRC etc. */
        pd_switch_to_rx_mode();
    }
}

static void pd_force_header_portrole_sink(uint8_t *frame /*>=2 bytes*/) {
    /* header byte1: bits: [Ext:1 NumDO:3 MsgID:3 PRRole:1]; enforce PRRole=0 */
    frame[1] &= ~0x01u;
}

/* Send a complete PD frame (header+payload without CRC). MessageID will be managed. */
static bool pd_send_frame_patch_header(const uint8_t *frame, uint8_t len) {
    if (len < 2 || len > 34) {
        cdc_acm_printf("! invalid PD length:%u\n", len);
        return false;
    }

    /* Copy and patch header: enforce sink PRRole and inject MsgID bits [1..3] */
    for (uint8_t i = 0; i < len; ++i) s_tx_buf[i] = frame[i];
    pd_force_header_portrole_sink(s_tx_buf);
    /* enforce SpecRev to selected */
    s_tx_buf[0] = (uint8_t)((s_tx_buf[0] & ~0xC0u) | (s_spec_rev == 3 ? 0x80u : 0x40u));
    s_tx_buf[1] = (s_tx_buf[1] & ~0x0Eu) | (s_tx_msg_id & 0x0Eu);

    /* Log our own TX message into the message buffer for CDC printing */
    save_message(PD_RX_SOP0, s_tx_buf, len);

    /* Enable TX_END interrupt to catch completion then switch to RX in ISR */
    USBPD->CONFIG |= IE_TX_END;

    /* Send SOP0 (non-blocking: do not busy wait here to avoid stalling CDC/UI) */
    pd_phy_send(false, s_tx_buf, len, UPD_SOP0);

    /* Increment MsgID immediately; ISR will flip back to RX on TX_END */
    s_tx_msg_id = (uint8_t)((s_tx_msg_id + 2) & 0x0Eu);

    return true;
}

void usb_pd_snk_enter(void) {
    /* Configure as SINK: internal Rd enabled on both CC, auto-ack as SINK */
    s_snk_active = true;
    s_tx_msg_id = 0;
    s_rdo_sent_once = false;

    /* Also enable external Rd control if present */
    usb_pd_cc_rd_en(true);

    USBPD->PORT_CC1 = CC_CMP_66 | CC_PD;
    USBPD->PORT_CC2 = CC_CMP_66 | CC_PD;

    /* Ensure we are in RX mode to start with */
    pd_switch_to_rx_mode();
    s_deferred_msgs |= 0x01; /* defer printing outside ISR */
}

void usb_pd_snk_exit(void) {
    /* Clear SNK runtime state and queues */
    s_snk_active = false;
    s_rdo_sent_once = false;
    s_pending_len = 0;
    s_tx_msg_id = 0;

    /* Reset auto-reply/EPR state */
    usb_pd_auto_reset();

    /* Drop any buffered messages/backlog on exit */
    clear_message_buffer();
    reset_message_counter();

    /* Remove internal Rd, keep comparator */
    USBPD->PORT_CC1 = CC_CMP_66;
    USBPD->PORT_CC2 = CC_CMP_66;
    /* Ensure CC drive is disabled */
    USBPD->PORT_CC1 &= ~CC_LVE;
    USBPD->PORT_CC2 &= ~CC_LVE;
    usb_pd_cc_rd_en(false);
    pd_switch_to_rx_mode();
    /* Only keep the exit print deferred flag */
    s_deferred_msgs = 0x02;
}

bool usb_pd_snk_is_active(void) { return s_snk_active; }

void usb_pd_snk_on_cdc_bytes(const uint8_t *data, uint8_t len) {
    if (!s_snk_active) return;

    /* Accept raw PD frame (header + payload, no CRC). */
    if (len == 0) return;

    if (len == 4 && data[0] == 'e' && data[1] == 'x' && data[2] == 'i' && data[3] == 't') {
        usb_pd_snk_exit();
        return;
    }

    /* Transmit */
    (void)pd_send_frame_patch_header(data, len);
}

bool usb_pd_snk_queue_frame(const uint8_t *frame, uint8_t len) {
    if (!s_snk_active) return false;
    if (len < 2 || len > sizeof(s_pending_frame)) return false;
    /* If PD TX is idle, send immediately to avoid waiting for next TX_END */
    if ((USBPD->CONTROL & PD_TX_EN) == 0) {
        (void)pd_send_frame_patch_header((const uint8_t *)frame, len);
        return true;
    }
    /* Otherwise, queue if empty */
    if (s_pending_len != 0) return false; /* only one-slot queue */
    for (uint8_t i = 0; i < len; ++i) s_pending_frame[i] = frame[i];
    s_pending_len = len;
    return true;
}

bool usb_pd_snk_on_tx_end_handle_pending(void) {
    if (s_pending_len == 0) return false;
    /* Send queued frame now (non-blocking) */
    (void)pd_send_frame_patch_header(s_pending_frame, s_pending_len);
    s_pending_len = 0;
    return true;
}

/* Build and send REQUEST for first PDO from received SRC_CAP frame */
bool usb_pd_snk_auto_request_on_src_cap(const uint8_t *rx_frame, uint8_t len) {
    if (!s_snk_active) return false;
    if (len < 6) return false;

    /* Parse header */
    uint8_t hdr0 = rx_frame[0];
    uint8_t hdr1 = rx_frame[1];
    uint8_t msg_type = (hdr0 & 0x1F);
    uint8_t ndo = (hdr1 >> 4) & 0x07;

    if (msg_type != DEF_TYPE_SRC_CAP || ndo == 0) {
        return false;
    }
    if (s_rdo_sent_once) {
        return false;
    }

    /* First PDO (little-endian) starts at rx_frame[2] */
    uint8_t rdo[4];
    rdo[0] = rx_frame[2];
    rdo[1] = rx_frame[3];
    rdo[2] = rx_frame[4];
    rdo[3] = rx_frame[5];

    /* Re-map into RDO as per WCH example logic to request PDO index 1 */
    rdo[3] = 0x03;              /* flags minimal + will OR object position */
    rdo[3] |= (1u << 4);        /* object position = 1 */
    rdo[1] = (rdo[1] & 0x03) | (rdo[0] << 2);
    rdo[2] = ((rdo[1] << 2) & 0x0C) | (rdo[0] >> 6);

    /* Build REQUEST frame: header + 1 DO */
    uint8_t frame[2 + 4];
    frame[0] = DEF_TYPE_REQUEST | 0x40; /* SpecRev=2.0, DataRole=UFP */
    frame[1] = 0x10;                    /* NumDO=1, PRRole=SNK(0), MsgID will be patched */
    frame[2] = rdo[0];
    frame[3] = rdo[1];
    frame[4] = rdo[2];
    frame[5] = rdo[3];

    s_deferred_msgs |= 0x04; /* defer printing outside ISR */
    bool ok = pd_send_frame_patch_header(frame, sizeof(frame));
    if (ok) s_rdo_sent_once = true;
    return ok;
}

void usb_pd_snk_poll(void) {
    uint8_t pending = s_deferred_msgs;
    if (!pending) return;
    if (!cdc_acm_is_configured()) return;

    if (pending & 0x01) {
        cdc_acm_printf("# enter SNK mode (%s): send raw PD frame bytes over CDC; send 'exit' to leave\n", (s_spec_rev==3?"PD3.0":"PD2.0"));
        s_deferred_msgs &= ~0x01;
    }
    if (pending & 0x02) {
        cdc_acm_prints("# exit SNK mode\n");
        s_deferred_msgs &= ~0x02;
    }
    if (pending & 0x04) {
        cdc_acm_prints("# auto REQUEST PDO1\n");
        s_deferred_msgs &= ~0x04;
    }
}

void usb_pd_snk_set_spec_rev(uint8_t rev) {
    if (rev == 3) s_spec_rev = 3; else s_spec_rev = 2;
}
uint8_t usb_pd_snk_get_spec_rev(void) { return s_spec_rev; }
uint8_t usb_pd_snk_get_spec_flag(void) { return (s_spec_rev == 3 ? 0x80u : 0x40u); }
