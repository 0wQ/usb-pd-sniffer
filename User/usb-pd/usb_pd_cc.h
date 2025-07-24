#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t cc_connection;        // 0:未连接，1:CC1, 2:CC2
    uint8_t cc_connection_count;  // 已连接计数
} cc_state_t;

void usb_pd_cc_en(bool en);
void usb_pd_cc_rd_en(bool en);

void usb_pd_cc_init(void);
void usb_pd_cc_check_connection(cc_state_t *cc_state);
void usb_pd_cc_detach(cc_state_t *cc_state);
