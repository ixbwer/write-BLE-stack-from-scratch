/*
 * BLE PHY 软件 TIFS Demo — 公共类型、常量与全局变量声明
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
#include <hal/nrf_timer.h>
#include <hal/nrf_ppi.h>

/* ---- Controller HAL 层头文件 ---- */
#include "hal/ccm.h"       /* struct ccm (需在 radio.h 之前) */
#include "hal/radio.h"     /* radio_phy_set, radio_aa_set, ... */

/* ---- Controller PDU 结构体定义 ---- */
#include "pdu_df.h"        /* struct pdu_cte_info */
#include "pdu_vendor.h"    /* Nordic 特定的 PDU 扩展 */
#include "pdu.h"           /* struct pdu_adv, PDU_AC_ACCESS_ADDR, ... */

/*===========================================================================
 * 配置参数
 *===========================================================================*/

#define ADV_INTERVAL_MS  100

/* T_IFS: BLE 规范定义的帧间间隔 = 150 μs */
#define T_IFS_US         150

/* RX ramp-up 延迟 (μs): RXEN → READY → START ≈ 40 μs */
#define RX_RAMP_US       40

/* TX ramp-up 延迟 (μs): TXEN → READY → START ≈ 40 μs */
#define TX_RAMP_US       40

/* RX chain delay (μs): 空口最后一个 bit → EVENTS_END 之间的内部处理延迟
 * nRF52832 1M PHY ≈ 10 μs (数据来源: nRF52832 Product Specification,
 * RADIO Timing, "Time from last bit on air to END event")
 * TX END 无此延迟: END 事件与最后 bit 对齐。 */
#define RX_CHAIN_DELAY_US  10

/* RX 超时 (μs) */
#define RX_TIMEOUT_US    500

/* 广播设备名称 */
#define ADV_NAME         "ZephyrSW"

/*===========================================================================
 * PPI 通道分配
 *
 * nRF52840 有 20 个可编程 PPI 通道 (0-19)。
 * 真实 Controller 使用通道 0-7, 我们使用高编号避免冲突。
 *===========================================================================*/
#define PPI_CH_TIMER_CLEAR  14   /* RADIO END → TIMER1 CLEAR */
#define PPI_CH_RXEN         15   /* TIMER1 CC[0] → RADIO RXEN */
#define PPI_CH_TXEN         16   /* TIMER1 CC[1] → RADIO TXEN */

/* Timer CC 通道 */
#define CC_IDX_RXEN  0   /* TX→RX 转换 */
#define CC_IDX_TXEN  1   /* RX→TX 转换 */

/*===========================================================================
 * 全局变量声明 (定义在 main.c)
 *===========================================================================*/

extern const uint8_t adv_addr[BDADDR_SIZE];

/* PDU 缓冲区 */
extern struct pdu_adv pdu_adv_ind;
extern struct pdu_adv pdu_scan_rsp;
extern struct pdu_adv pdu_rx_buf;

/* 统计计数器 */
extern volatile uint32_t adv_event_count;
extern volatile uint32_t scan_req_count;
extern volatile uint32_t scan_rsp_count;

#endif /* BLE_COMMON_H */
