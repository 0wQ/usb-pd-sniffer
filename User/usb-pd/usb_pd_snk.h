#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Enter SNK active mode on CC, auto-GoodCRC on RX */
void usb_pd_snk_enter(void);
/* Exit SNK mode back to passive listen */
void usb_pd_snk_exit(void);
/* Query if SNK mode is active */
bool usb_pd_snk_is_active(void);
/* Handle CDC-received raw bytes while in SNK mode (either PD frame or command) */
void usb_pd_snk_on_cdc_bytes(const uint8_t *data, uint8_t len);

/* Auto: when a SRC_CAP is received in SNK mode, send a REQUEST for PDO#1 */
bool usb_pd_snk_auto_request_on_src_cap(const uint8_t *rx_frame, uint8_t len);

/* Periodic poll from main loop to flush deferred prints etc. (non-ISR context) */
void usb_pd_snk_poll(void);

/* Queue a PD frame (header+payload, no CRC) to be sent immediately after the current GoodCRC TX_END. */
bool usb_pd_snk_queue_frame(const uint8_t *frame, uint8_t len);

/* Called from IRQ on IF_TX_END to continue with any pending auto reply; returns true if a frame was started. */
bool usb_pd_snk_on_tx_end_handle_pending(void);

/* Set/Get selected PD Specification Revision for outgoings: 2 or 3 */
void usb_pd_snk_set_spec_rev(uint8_t rev);
uint8_t usb_pd_snk_get_spec_rev(void);
/* Return header SpecRev bits mask to OR into byte0: 0x40 for PD2.0, 0x80 for PD3.0 */
uint8_t usb_pd_snk_get_spec_flag(void);
