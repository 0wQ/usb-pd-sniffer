#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "ch32x035.h"
#include "usbd_core.h"
#include "usbd_cdc_acm.h"

void cdc_acm_init(uint8_t busid, uintptr_t reg_base);
void cdc_acm_prints(char *str);
void cdc_acm_printf(char *format, ...);
uint8_t cdc_acm_get_dtr(void);
bool cdc_acm_is_configured(void);
