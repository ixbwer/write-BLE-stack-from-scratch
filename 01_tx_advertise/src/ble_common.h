/*
 * BLE PHY Raw TX Demo — 公共类型、常量与全局变量声明
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BLE_COMMON_H
#define BLE_COMMON_H

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <soc.h>
#include <hal/nrf_clock.h>

/* ---- Controller HAL 层头文件 ---- */
#include "hal/ccm.h"       /* struct ccm (需在 radio.h 之前) */
#include "hal/radio.h"     /* radio_phy_set, radio_aa_set, ... */

/* ---- Controller PDU 结构体定义 ---- */
#include "pdu_df.h"        /* struct pdu_cte_info */
#include "pdu_vendor.h"    /* Nordic 特定的 PDU 扩展 */
#include "pdu.h"           /* struct pdu_adv, PDU_AC_ACCESS_ADDR, ... */

/*===========================================================================
 * 全局变量声明 (定义在 main.c)
 *===========================================================================*/

extern const uint8_t adv_addr[BDADDR_SIZE];
extern struct pdu_adv pdu;

#endif /* BLE_COMMON_H */
