/*
 * BLE PHY Connection Loop Demo — 公共类型、常量与全局变量声明
 *
 * 相比 04_phy_first_anchor 新增:
 *   - anchor_tracker 扩展: consecutive_misses, anchor_update_count 等
 *   - conn_event_rx_crc_err 统计
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
#define WIN_DELAY_LEGACY            1250  /* transmitWindowDelay (μs) */
#define CONN_INT_UNIT_US            1250  /* connInterval unit (1.25ms) */
#define EVENT_JITTER_US             16    /* Active clock jitter margin (μs) */
#define EVENT_TICKER_RES_MARGIN_US  31    /* RTC tick resolution ~30.52μs ceil */
#define ADDR_US_1M                  40    /* Preamble(8)+AA(32) for 1M PHY */
#define RTC0_CMP_OFFSET_MIN         3     /* NRF_RTC0 CC 最小比较偏移 */

#define ADV_NAME         "ConLoop"

/* TX/RX ramp-up 延迟 (μs): fast ramp-up ≈ 40 μs */
#define RX_RAMP_US       40
#define TX_RAMP_US       40

/* Chain delay: RADIO END 事件相对于实际空中包结束的延迟
 * nRF52832 1M PHY: RX chain delay = 9.4μs (ceil→10), TX chain delay = 0.6μs (ceil→1) */
#define RX_CHAIN_DELAY_1M_US  10
#define TX_CHAIN_DELAY_US      1

/*===========================================================================
 * PPI 通道分配 (SW TIFS)
 *===========================================================================*/
#define PPI_CH_TIMER_CLEAR  14   /* RADIO END → TIMER1 CLEAR */
#define PPI_CH_RXEN         15   /* TIMER1 CC[0] → RADIO RXEN */
#define PPI_CH_TXEN         16   /* TIMER1 CC[1] → RADIO TXEN */

#define CC_IDX_RXEN  0   /* Timer CC[0]: TX→RX 转换 */
#define CC_IDX_TXEN  1   /* Timer CC[1]: RX→TX 转换 */

#define LOCAL_SCA_PPM 50

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
 * 锚点追踪结构体
 *
 * 相比 04: 增加了 consecutive_misses 和 anchor_update_count
 * (为 supervision timeout 和统计用)
 *===========================================================================*/
struct anchor_tracker {
	uint32_t initial_anchor_rtc;     /* 首次计算出的锚点 RTC tick */
	uint32_t initial_anchor_us;      /* 首次锚点的 μs 偏移量 */
	uint32_t current_anchor_rtc;     /* 当前锚点 RTC tick */
	uint32_t consecutive_misses;     /* 连续 RX 失败次数 */
	uint32_t max_consecutive_misses; /* 历史最大连续失败 */
	uint32_t anchor_update_count;    /* 锚点更新次数 (= 成功 RX 次数) */
};

/*===========================================================================
 * 全局变量声明 (定义在 main.c)
 *===========================================================================*/

extern const uint8_t adv_addr[BDADDR_SIZE];
extern const uint8_t adv_data[];
#define ADV_DATA_LEN 12  /* sizeof(adv_data): 3 (flags) + 9 (name "ConLoop") */

extern struct conn_param conn_params;

/* 连接态变量 */
extern uint8_t tx_sn;
extern uint8_t rx_nesn;
extern uint16_t conn_event_counter;
extern uint8_t last_unmapped_chan;

extern struct pdu_data tx_pdu_buf;

/* PDU 缓冲区 */
extern struct pdu_adv pdu_adv_ind;
extern struct pdu_adv pdu_adv_rx;
extern struct pdu_data pdu_data_rx;

/* CONNECT_IND 接收时刻 */
extern uint32_t connect_end_rtc;

/* SCA PPM 映射表 */
extern const uint16_t sca_ppm_table[];

/* 锚点追踪 */
extern struct anchor_tracker anchor;

/* 统计 */
extern volatile uint32_t adv_event_count;
extern volatile uint32_t conn_event_rx_ok;
extern volatile uint32_t conn_event_rx_timeout;
extern volatile uint32_t conn_event_rx_crc_err;

#endif /* BLE_COMMON_H */
