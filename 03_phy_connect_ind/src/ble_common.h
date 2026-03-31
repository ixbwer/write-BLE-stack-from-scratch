/*
 * BLE PHY CONNECT_IND Demo — 公共类型、常量与全局变量声明
 *
 * 本例程在 02_tifs_scan 基础上增加:
 *   - struct conn_param: CONNECT_IND 解析结果
 *   - RTC0 时间戳: 用于捕获 CONNECT_IND 接收时刻 (T0)
 *   - ADV 逻辑从 SCAN_REQ/SCAN_RSP 简化为只等待 CONNECT_IND
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
#include "hal/ccm.h"
#include "hal/radio.h"

/* ---- Controller PDU 结构体定义 ---- */
#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

/* ---- Zephyr 控制器 Ticker 定时层 ---- */
#include "hal/ticker.h"

/*===========================================================================
 * 配置参数
 *===========================================================================*/

/* 广播间隔 (ms) */
#define ADV_INTERVAL_MS  100

/* T_IFS = 150 μs */
#define T_IFS_US         150

/* ADV RX 超时 (μs) */
#define ADV_RX_TIMEOUT_US  600

/* BLE 定时常量 */
#define CONN_INT_UNIT_US  1250  /* connInterval unit (1.25ms) */

#define ADV_NAME         "ConnInd"

/* TX/RX ramp-up 延迟 (μs): fast ramp-up ≈ 40 μs */
#define RX_RAMP_US       40
#define TX_RAMP_US       40

/*===========================================================================
 * PPI 通道分配 (SW TIFS)
 *===========================================================================*/
#define PPI_CH_TIMER_CLEAR  14   /* RADIO END → TIMER1 CLEAR */
#define PPI_CH_RXEN         15   /* TIMER1 CC[0] → RADIO RXEN */
#define PPI_CH_TXEN         16   /* TIMER1 CC[1] → RADIO TXEN */

#define CC_IDX_RXEN  0   /* Timer CC[0]: TX→RX 转换 */
#define CC_IDX_TXEN  1   /* Timer CC[1]: RX→TX 转换 */

/*===========================================================================
 * 连接参数结构体
 *===========================================================================*/
struct conn_param {
	uint8_t  access_addr[4];
	uint8_t  crc_init[3];
	uint8_t  win_size;
	uint16_t win_offset;
	uint16_t interval;
	uint16_t latency;
	uint16_t timeout;
	uint8_t  chan_map[PDU_CHANNEL_MAP_SIZE];
	uint8_t  hop;
	uint8_t  sca;
	uint8_t  chan_count;
	uint8_t  peer_addr[BDADDR_SIZE];
};

/*===========================================================================
 * 全局变量声明 (定义在 main.c)
 *===========================================================================*/

extern const uint8_t adv_addr[BDADDR_SIZE];
extern const uint8_t adv_data[];
#define ADV_DATA_LEN 14  /* sizeof(adv_data) */

extern struct conn_param conn_params;

/* PDU 缓冲区 */
extern struct pdu_adv pdu_adv_ind;
extern struct pdu_adv pdu_adv_rx;

/* CONNECT_IND 接收时刻 (NRF_RTC0 Counter) — 这就是 "T0", 锚点的起始基准 */
extern uint32_t connect_end_rtc;

/* SCA PPM 映射表 */
extern const uint16_t sca_ppm_table[];

/* 统计 */
extern volatile uint32_t adv_event_count;

#endif /* BLE_COMMON_H */
