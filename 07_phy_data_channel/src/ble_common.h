/*
 * BLE PHY Data Channel Demo — 公共类型、常量与全局变量声明
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

#define ADV_INTERVAL_MS  100
#define T_IFS_US         150
#define ADV_RX_TIMEOUT_US  600

/* BLE 定时常量 */
#define WIN_DELAY_LEGACY            1250
#define CONN_INT_UNIT_US            1250
#define EVENT_JITTER_US             16
#define EVENT_TICKER_RES_MARGIN_US  31
#define ADDR_US_1M                  40
#define RTC0_CMP_OFFSET_MIN         3

#define ADV_NAME         "DataCh"

#define RX_RAMP_US       40
#define TX_RAMP_US       40

#define RX_CHAIN_DELAY_1M_US  10
#define TX_CHAIN_DELAY_US      1

/*===========================================================================
 * PPI 通道分配 (SW TIFS)
 *===========================================================================*/
#define PPI_CH_TIMER_CLEAR  14
#define PPI_CH_RXEN         15
#define PPI_CH_TXEN         16

#define CC_IDX_RXEN  0
#define CC_IDX_TXEN  1

#define LOCAL_SCA_PPM 50

/*===========================================================================
 * L2CAP 常量
 *===========================================================================*/
#define L2CAP_CID_ATT       0x0004  /* Attribute Protocol */
#define L2CAP_CID_SIG       0x0005  /* L2CAP Signaling */
#define L2CAP_CID_SMP       0x0006  /* Security Manager */
#define L2CAP_HDR_SIZE      4       /* Length(2) + CID(2) */
#define L2CAP_MAX_PAYLOAD   512     /* 最大 L2CAP SDU */

/* LL Data PDU 默认最大 payload = 27 字节 (BLE 4.0/4.1) */
#define LL_DATA_MTU_DEFAULT 27

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
 *===========================================================================*/
struct anchor_tracker {
	uint32_t initial_anchor_rtc;
	uint32_t initial_anchor_us;
	uint32_t current_anchor_rtc;
	uint32_t last_rx_anchor_rtc;
	uint32_t ww_current_us;
	uint32_t ww_periodic_us;
	uint32_t ww_max_us;
	uint32_t win_size_event_us;
	uint32_t anchor_update_count;
	uint32_t consecutive_misses;
	uint32_t max_consecutive_misses;
};

/*===========================================================================
 * L2CAP 重组缓冲区
 *===========================================================================*/
struct l2cap_reassembly {
	uint8_t  buf[L2CAP_MAX_PAYLOAD + L2CAP_HDR_SIZE];
	uint16_t expected_len;  /* L2CAP Length 字段值 + 4 (header) */
	uint16_t received_len;  /* 已接收字节数 */
	bool     in_progress;   /* 正在重组 */
};

/*===========================================================================
 * 数据 TX 队列 (简单环形缓冲)
 *===========================================================================*/
#define TX_QUEUE_SIZE  4

struct tx_frag {
	uint8_t  data[LL_DATA_MTU_DEFAULT];
	uint8_t  len;
	uint8_t  llid;  /* PDU_DATA_LLID_DATA_START or _CONTINUE */
};

struct tx_queue {
	struct tx_frag frags[TX_QUEUE_SIZE];
	uint8_t head;
	uint8_t tail;
	uint8_t count;
};

/*===========================================================================
 * 全局变量声明 (定义在 main.c)
 *===========================================================================*/

extern const uint8_t adv_addr[BDADDR_SIZE];
extern const uint8_t adv_data[];
#define ADV_DATA_LEN 14

extern struct conn_param conn_params;

/* 连接态变量 */
extern uint8_t tx_sn;
extern uint8_t rx_nesn;
extern uint16_t conn_event_counter;
extern uint8_t last_unmapped_chan;
extern volatile bool conn_terminated;

extern struct pdu_data tx_pdu_buf;
extern bool tx_pdu_pending;

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

/* Drift correction */
extern uint32_t last_rx_aa_us;
extern uint32_t last_rx_ready_us;

/* L2CAP 重组 */
extern struct l2cap_reassembly l2cap_reasm;

/* 数据 TX 队列 */
extern struct tx_queue data_tx_q;

/* 数据统计 */
extern uint32_t l2cap_rx_count;
extern uint32_t l2cap_tx_count;

/* L2CAP 延迟日志 (在 conn_event 结束后打印, 避免阻塞 TIFS) */
extern uint16_t l2cap_last_rx_cid;
extern uint16_t l2cap_last_rx_len;
extern int      l2cap_last_tx_frags;
extern bool     l2cap_last_rx_pending;

#endif /* BLE_COMMON_H */
