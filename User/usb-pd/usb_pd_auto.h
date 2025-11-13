#pragma once

#include <stdint.h>

/* Inspect a just-received PD frame and, if needed, queue an automatic reply.
 * Should be called from IRQ context after RX_ACT, ideally right after scheduling GoodCRC. */
void usb_pd_auto_on_rx(uint32_t status, const uint8_t *data, uint8_t len);

/* Periodic tasks for auto replies (e.g., EPR keepalive); call in main loop */
void usb_pd_auto_poll(void);
/* Reset auto-reply/EPR states */
void usb_pd_auto_reset(void);
