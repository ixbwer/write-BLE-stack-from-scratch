/*
 * BLE PHY Slave Latency Demo — 公共类型、常量与全局变量声明
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

#include "hal/ccm.h"
#include "hal/radio.h"

#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

#include "hal/ticker.h"

/*===========================================================================
 * 配置参数
 *===========================================================================*/

#define ADV_INTERVAL_MS  100
#define T_IFS_US         150
#define ADV_RX_TIMEOUT_US  600

#define WIN_DELAY_LEGACY            1250
#define CONN_INT_UNIT_US            1250
#define EVENT_JITTER_US             16
#define EVENT_TICKER_RES_MARGIN_US  31
#define ADDR_US_1M                  40
#define RTC0_CMP_OFFSET_MIN         3

#define ADV_NAME         "SLat"

#define RX_RAMP_US       40
#define TX_RAMP_US       40

#define RX_CHAIN_DELAY_1M_US  10
#define TX_CHAIN_DELAY_US      1

#define PPI_CH_TIMER_CLEAR  14
#define PPI_CH_RXEN         15
#define PPI_CH_TXEN         16

#define CC_IDX_RXEN  0
#define CC_IDX_TXEN  1

#define LOCAL_SCA_PPM 50

/* L2CAP */
#define L2CAP_CID_ATT       0x0004
#define L2CAP_CID_SIG       0x0005
#define L2CAP_CID_SMP       0x0006
#define L2CAP_HDR_SIZE      4
#define L2CAP_MAX_PAYLOAD   512
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
 * Slave Latency 追踪
 *===========================================================================*/
struct latency_tracker {
	uint16_t latency_max;      /* connSlaveLatency from CONNECT_IND */
	uint16_t latency_used;     /* 当前连续跳过的事件数 */
	uint32_t events_skipped;   /* 总跳过事件数 (统计) */
	uint32_t events_listened;  /* 总监听事件数 (统计) */
};

/*===========================================================================
 * L2CAP 重组缓冲区
 *===========================================================================*/
struct l2cap_reassembly {
	uint8_t  buf[L2CAP_MAX_PAYLOAD + L2CAP_HDR_SIZE];
	uint16_t expected_len;
	uint16_t received_len;
	bool     in_progress;
};

#define TX_QUEUE_SIZE  4

struct tx_frag {
	uint8_t  data[LL_DATA_MTU_DEFAULT];
	uint8_t  len;
	uint8_t  llid;
};

struct tx_queue {
	struct tx_frag frags[TX_QUEUE_SIZE];
	uint8_t head;
	uint8_t tail;
	uint8_t count;
};

/*===========================================================================
 * 全局变量声明
 *===========================================================================*/

extern const uint8_t adv_addr[BDADDR_SIZE];
extern const uint8_t adv_data[];
#define ADV_DATA_LEN 12

extern struct conn_param conn_params;

extern uint8_t tx_sn;
extern uint8_t rx_nesn;
extern uint16_t conn_event_counter;
extern uint8_t last_unmapped_chan;
extern volatile bool conn_terminated;

extern struct pdu_data tx_pdu_buf;
extern bool tx_pdu_pending;

extern struct pdu_adv pdu_adv_ind;
extern struct pdu_adv pdu_adv_rx;
extern struct pdu_data pdu_data_rx;

extern uint32_t connect_end_rtc;
extern const uint16_t sca_ppm_table[];
extern struct anchor_tracker anchor;
extern struct latency_tracker lat;

extern volatile uint32_t adv_event_count;
extern volatile uint32_t conn_event_rx_ok;
extern volatile uint32_t conn_event_rx_timeout;
extern volatile uint32_t conn_event_rx_crc_err;

extern uint32_t last_rx_aa_us;
extern uint32_t last_rx_ready_us;

extern struct l2cap_reassembly l2cap_reasm;
extern struct tx_queue data_tx_q;
extern uint32_t l2cap_rx_count;
extern uint32_t l2cap_tx_count;

#endif /* BLE_COMMON_H */
