#include "usb_pd_auto.h"

#include "usb_pd_snk.h"
#include "ch32x035_usbpd.h"
#include "millis.h"

/* Helpers for parsing */
static inline uint8_t pd_msg_type(const uint8_t *f) { return f[0] & 0x1F; }
static inline uint8_t pd_num_do(const uint8_t *f) { return (f[1] >> 4) & 0x07; }
static inline uint8_t pd_is_extended(const uint8_t *f) { return (f[1] & 0x80) != 0; }

#define EXT_TYPE_EPR_SOURCE_CAP 0x11
#define EXT_TYPE_EXT_CONTROL    0x10

#define ECDB_EPR_KEEPALIVE      0x03
#define ECDB_EPR_KEEPALIVE_ACK  0x04

static volatile uint8_t s_epr_req_sent = 0;
static volatile uint8_t s_epr_active = 0;
static volatile uint32_t s_keepalive_due_ms = 0;
static const uint32_t KEEPALIVE_PERIOD_MS = 500; /* default 500ms */

static uint8_t build_ext_control_keepalive(uint8_t *out) {
    /* Header(Ext=1, NDO=1, MsgType=ExtControl), ExtHdr(Chunked=1, Chunk#=0, ReqChunk=0, DataSize=2), Data: [0x03,0x00] */
    out[0] = EXT_TYPE_EXT_CONTROL; /* SpecRev patched at TX */
    out[1] = 0x80 | 0x10; /* Ext=1, NumDO=1 */
    uint16_t eh = (uint16_t)(0x8000 | 0x0002); /* 0x8002 */
    out[2] = (uint8_t)(eh & 0xFF);
    out[3] = (uint8_t)(eh >> 8);
    out[4] = ECDB_EPR_KEEPALIVE;
    out[5] = 0x00;
    return 6;
}

/* Build REQUEST for PDO#1 from received SRC_CAP frame. Returns frame length or 0 on failure. */
static uint8_t build_request_pdo1_from_srccap(const uint8_t *src_cap, uint8_t len, uint8_t *out) {
    if (len < 6) return 0; /* header + at least 1 DO */
    if (pd_is_extended(src_cap)) return 0; /* not extended */
    if (pd_num_do(src_cap) == 0) return 0;

    /* Build RDO as per WCH sample mapping */
    uint8_t rdo[4];
    rdo[0] = src_cap[2];
    rdo[1] = src_cap[3];
    rdo[2] = src_cap[4];
    rdo[3] = src_cap[5];
    rdo[3] = 0x03;              /* minimal flags */
    rdo[3] |= (1u << 4);        /* object position = 1 */
    rdo[1] = (rdo[1] & 0x03) | (rdo[0] << 2);
    rdo[2] = ((rdo[1] << 2) & 0x0C) | (rdo[0] >> 6);

    /* PD3.0: set RDO bit23 (UEM supported) and bit22 (EPR capable) based on PDO bit24/bit23 respectively */
    if (usb_pd_snk_get_spec_rev() == 3) {
        /* PDO bit24 -> src_cap[5] bit0; PDO bit23 -> src_cap[4] bit7 */
        uint8_t pdo_uem = (uint8_t)(src_cap[5] & 0x01);      /* 1 if UEM supported */
        uint8_t pdo_epr = (uint8_t)(src_cap[4] & 0x80) >> 7; /* 1 if EPR capable */
        /* RDO bit23/bit22 are rdo[2] bit7/bit6 */
        rdo[2] = (uint8_t)((rdo[2] & ~(uint8_t)0xC0) | (pdo_uem ? 0x80 : 0) | (pdo_epr ? 0x40 : 0));
    }

    /* Build REQUEST frame header + 1 DO; SpecRev set to 2.0 here */
    out[0] = 0x02 /*Request*/ | 0x40; /* MsgType + SpecRev=2.0 */
    out[1] = 0x10; /* NumDO=1, PRRole=SNK(0), MsgID will be patched later */
    out[2] = rdo[0]; out[3] = rdo[1]; out[4] = rdo[2]; out[5] = rdo[3];
    return 6;
}

void usb_pd_auto_on_rx(uint32_t status, const uint8_t *data, uint8_t len) {
    if (!usb_pd_snk_is_active()) return;
    if ((status & MASK_PD_STAT) != PD_RX_SOP0) return; /* only SOP0 */
    if (len < 6) return;

    uint8_t type = pd_msg_type(data);
    uint8_t ext = pd_is_extended(data);

    /* Rule 1: On SourceCap, auto send REQUEST for PDO#1 (once per attach). */
    if (!ext && type == DEF_TYPE_SRC_CAP) {
        uint8_t frame[6];
        uint8_t flen = build_request_pdo1_from_srccap(data, len, frame);
        if (flen) {
            (void)usb_pd_snk_queue_frame(frame, flen);
        }
        return;
    }

    /* Rule 2: EPR Source Capabilities handling (chunked). */
    if (ext && type == EXT_TYPE_EPR_SOURCE_CAP) {
        /* Only meaningful when operating as PD3.0 */
        if (usb_pd_snk_get_spec_rev() != 3) return;
        if (len < 4) return; /* need ext header */
        uint16_t ext_hdr = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
        uint8_t chunked = (ext_hdr >> 15) & 0x01;
        uint8_t chunk_no = (ext_hdr >> 11) & 0x0F;
        uint8_t req_chunk = (ext_hdr >> 10) & 0x01;

        if (chunked && !req_chunk && chunk_no == 0) {
            /* Send a Chunk Request asking for chunk #1
             * Build: Header(Ext=1, NDO=1, Type=EPRSourceCap, SpecRev=3.0),
             *        ExtHdr(Chunked=1, Chunk#=1, ReqChunk=1, DataSize=0),
             *        Pad to 4B boundary for the data block (ExtHdr+Data) */
            uint8_t frame[6];
            frame[0] = (EXT_TYPE_EPR_SOURCE_CAP); /* MsgType; SpecRev will be enforced in TX path */
            frame[1] = 0x80 | 0x10; /* Ext=1, NumDO=1, PRRole=0, MsgID patched later */
            uint16_t eh = (uint16_t)(0x8000 | 0x0800 | 0x0400); /* 0x8C00 */
            frame[2] = (uint8_t)(eh & 0xFF);
            frame[3] = (uint8_t)(eh >> 8);
            frame[4] = 0x00; /* padding to make (ExtHdr+Data) length multiple of 4 */
            frame[5] = 0x00;
            (void)usb_pd_snk_queue_frame(frame, sizeof(frame));
            return;
        }

        if (chunked && !req_chunk && chunk_no == 1) {
            /* After receiving chunk #1, send EPR_Request (sample payload provided by user) */
            uint8_t frame[10];
            frame[0] = 0x09; /* EPR_REQUEST (data msg), SpecRev will be enforced in TX path */
            frame[1] = 0x20;       /* NumDO=2, PRRole=0, MsgID patched later */
            /* Copy sample DOs: 8 bytes */
            frame[2] = 0x2C; frame[3] = 0xB1; frame[4] = 0x44; frame[5] = 0x13;
            frame[6] = 0x2C; frame[7] = 0x91; frame[8] = 0x81; frame[9] = 0x0A;
            (void)usb_pd_snk_queue_frame(frame, sizeof(frame));
            s_epr_req_sent = 1;
            return;
        }

        return;
    }

    /* Rule 3: After EPR_Request, when PS_RDY received, send ExtControl KeepAlive and mark active */
    if (!ext && type == DEF_TYPE_PS_RDY) {
        if (usb_pd_snk_get_spec_rev() == 3 && s_epr_req_sent) {
            uint8_t frame[6];
            uint8_t flen = build_ext_control_keepalive(frame);
            (void)usb_pd_snk_queue_frame(frame, flen);
            s_epr_active = 1;
            s_keepalive_due_ms = millis() + KEEPALIVE_PERIOD_MS;
        }
        return;
    }

    /* Rule 4: Upon receiving SRC ExtControl KeepAlive_Ack, schedule next KeepAlive after period */
    if (ext && type == EXT_TYPE_EXT_CONTROL) {
        if (usb_pd_snk_get_spec_rev() != 3) return;
        if (len < 6) return; /* need at least ext hdr + 2-byte data */
        uint16_t ext_hdr = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
        uint8_t data_size = ext_hdr & 0x01FF;
        if (data_size < 2) return;
        uint8_t ecdb_type = data[4];
        if (ecdb_type == ECDB_EPR_KEEPALIVE_ACK) {
            s_epr_active = 1;
            s_keepalive_due_ms = millis() + KEEPALIVE_PERIOD_MS;
        }
        return;
    }
}

void usb_pd_auto_poll(void) {
    if (!s_epr_active) return;
    if (usb_pd_snk_get_spec_rev() != 3) return;
    uint32_t now = millis();
    if (s_keepalive_due_ms != 0 && (int32_t)(now - s_keepalive_due_ms) >= 0) {
        uint8_t frame[6];
        uint8_t flen = build_ext_control_keepalive(frame);
        if (usb_pd_snk_queue_frame(frame, flen)) {
            s_keepalive_due_ms = now + KEEPALIVE_PERIOD_MS;
        }
    }
}

void usb_pd_auto_reset(void) {
    s_epr_req_sent = 0;
    s_epr_active = 0;
    s_keepalive_due_ms = 0;
}
