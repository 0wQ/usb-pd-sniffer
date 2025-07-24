/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CHERRYUSB_CONFIG_H
#define CHERRYUSB_CONFIG_H

/* ================ USB common Configuration ================ */

#ifdef __RTTHREAD__
#include <rtthread.h>

#define CONFIG_USB_PRINTF(...) rt_kprintf(__VA_ARGS__)
#else
// #define CONFIG_USB_PRINTF(...) printf(__VA_ARGS__)
#define CONFIG_USB_PRINTF(...) {}
#endif

#ifndef CONFIG_USB_DBG_LEVEL
#define CONFIG_USB_DBG_LEVEL USB_DBG_INFO
#endif

/* Enable print with color */
#define CONFIG_USB_PRINTF_COLOR_ENABLE

// #define CONFIG_USB_DCACHE_ENABLE

/* data align size when use dma or use dcache */
#ifdef CONFIG_USB_DCACHE_ENABLE
#define CONFIG_USB_ALIGN_SIZE 32 // 32 or 64
#else
#define CONFIG_USB_ALIGN_SIZE 4
#endif

/* attribute data into no cache ram */
#define USB_NOCACHE_RAM_SECTION __attribute__((section(".noncacheable")))

/* use usb_memcpy default for high performance but cost more flash memory.
 * And, arm libc has a bug that memcpy() may cause data misalignment when the size is not a multiple of 4.
*/
// #define CONFIG_USB_MEMCPY_DISABLE

/* ================= USB Device Stack Configuration ================ */

/* Ep0 in and out transfer buffer */
#ifndef CONFIG_USBDEV_REQUEST_BUFFER_LEN
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN 512
#endif

/* Setup packet log for debug */
// #define CONFIG_USBDEV_SETUP_LOG_PRINT

/* Send ep0 in data from user buffer instead of copying into ep0 reqdata
 * Please note that user buffer must be aligned with CONFIG_USB_ALIGN_SIZE
*/
// #define CONFIG_USBDEV_EP0_INDATA_NO_COPY

/* Check if the input descriptor is correct */
// #define CONFIG_USBDEV_DESC_CHECK

/* Enable test mode */
// #define CONFIG_USBDEV_TEST_MODE

/* enable advance desc register api */
#define CONFIG_USBDEV_ADVANCE_DESC

/* move ep0 setup handler from isr to thread */
// #define CONFIG_USBDEV_EP0_THREAD

#ifndef CONFIG_USBDEV_EP0_PRIO
#define CONFIG_USBDEV_EP0_PRIO 4
#endif

#ifndef CONFIG_USBDEV_EP0_STACKSIZE
#define CONFIG_USBDEV_EP0_STACKSIZE 2048
#endif

/* ================ USB Device Port Configuration ================*/

#ifndef CONFIG_USBDEV_MAX_BUS
#define CONFIG_USBDEV_MAX_BUS 1 // for now, bus num must be 1 except hpm ip
#endif

#ifndef CONFIG_USBDEV_EP_NUM
#define CONFIG_USBDEV_EP_NUM 8
#endif

/* ---------------- EHCI Configuration ---------------- */

#define CONFIG_USB_EHCI_HCCR_OFFSET     (0x0)
#define CONFIG_USB_EHCI_FRAME_LIST_SIZE 1024
#define CONFIG_USB_EHCI_QH_NUM          CONFIG_USBHOST_PIPE_NUM
#define CONFIG_USB_EHCI_QTD_NUM         (CONFIG_USB_EHCI_QH_NUM * 3)
#define CONFIG_USB_EHCI_ITD_NUM         4
// #define CONFIG_USB_EHCI_HCOR_RESERVED_DISABLE
// #define CONFIG_USB_EHCI_CONFIGFLAG
// #define CONFIG_USB_EHCI_ISO
// #define CONFIG_USB_EHCI_WITH_OHCI
// #define CONFIG_USB_EHCI_DESC_DCACHE_ENABLE

/* ---------------- OHCI Configuration ---------------- */
#define CONFIG_USB_OHCI_HCOR_OFFSET (0x0)
#define CONFIG_USB_OHCI_ED_NUM CONFIG_USBHOST_PIPE_NUM
#define CONFIG_USB_OHCI_TD_NUM 3
// #define CONFIG_USB_OHCI_DESC_DCACHE_ENABLE

/* ---------------- XHCI Configuration ---------------- */
#define CONFIG_USB_XHCI_HCCR_OFFSET (0x0)

/* ---------------- DWC2 Configuration ---------------- */
/* largest non-periodic USB packet used / 4 */
// #define CONFIG_USB_DWC2_NPTX_FIFO_SIZE (512 / 4)
/* largest periodic USB packet used / 4 */
// #define CONFIG_USB_DWC2_PTX_FIFO_SIZE (1024 / 4)
/*
 * (largest USB packet used / 4) + 1 for status information + 1 transfer complete +
 * 1 location each for Bulk/Control endpoint for handling NAK/NYET scenario
 */
// #define CONFIG_USB_DWC2_RX_FIFO_SIZE ((1012 - CONFIG_USB_DWC2_NPTX_FIFO_SIZE - CONFIG_USB_DWC2_PTX_FIFO_SIZE))

/* ---------------- MUSB Configuration ---------------- */
// #define CONFIG_USB_MUSB_SUNXI

#ifndef usb_phyaddr2ramaddr
#define usb_phyaddr2ramaddr(addr) (addr)
#endif

#ifndef usb_ramaddr2phyaddr
#define usb_ramaddr2phyaddr(addr) (addr)
#endif

#endif