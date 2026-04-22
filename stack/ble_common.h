/*
 * BLE PHY Encryption Demo — 公共类型、常量与全局变量声明
 *
 * 相比 12_phy_l2cap_sig 的变化:
 *   1. 新增 LE Encryption 状态机和加密状态结构体
 *   2. 新增 CCM/ECB 加密缓冲区
 *   3. 新增加密特性位声明 (LE Encryption)
 *   4. MAXLEN 增大至 31 以容纳 MIC (4 字节)
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

/* BLE 4.x default LL Data PDU payload capacity */
#define LL_PAYLOAD_OCTETS_MAX       27

#ifndef ADV_NAME
#define ADV_NAME         "GATT"
#endif

/* Encryption */
#define PDU_MIC_SIZE     4

/* Slave-initiated procedure: 连接后多久触发 (秒) */
#define SLAVE_CONN_PARAM_REQ_DELAY_S   10
#define SLAVE_TERMINATE_DELAY_S        30

#define RX_RAMP_US       40
#define TX_RAMP_US       40

#define RX_CHAIN_DELAY_1M_US  10
#define RX_CHAIN_DELAY_US     RX_CHAIN_DELAY_1M_US
#define TX_CHAIN_DELAY_US      1

/* ★ Multi-PDU: 单个连接事件内最大 RX/TX 交互次数 */
#define MAX_EXCHANGES_PER_EVENT  6

#ifndef BLE_FEATURE_MULTI_PDU
#define BLE_FEATURE_MULTI_PDU 1
#endif

#ifndef BLE_FEATURE_ATT
#define BLE_FEATURE_ATT 1
#endif

#ifndef BLE_FEATURE_SMP
#define BLE_FEATURE_SMP 1
#endif

#ifndef BLE_FEATURE_SLAVE_CPR
#define BLE_FEATURE_SLAVE_CPR 1
#endif

#ifndef BLE_FEATURE_SLAVE_TERMINATE
#define BLE_FEATURE_SLAVE_TERMINATE 1
#endif

#ifndef BLE_FEATURE_GATT_NOTIFY
#define BLE_FEATURE_GATT_NOTIFY 1
#endif

/* 07+: Slave Latency 事件跳过机制 */
#ifndef BLE_FEATURE_SLAVE_LATENCY
#define BLE_FEATURE_SLAVE_LATENCY 1
#endif

/* 08+: Slave 主动 LL 过程、上层栈 (L2CAP/SMP/ATT) 初始化、GATT notify */
#ifndef BLE_FEATURE_LL_PROCEDURE
#define BLE_FEATURE_LL_PROCEDURE 1
#endif

/* TX→RX 切换补偿: T_IFS 减去 RX 预热时间和 TX 链延迟 */
#define TX_TO_RX_CC_US  (T_IFS_US - RX_RAMP_US - TX_CHAIN_DELAY_US)

#define PPI_CH_TIMER_CLEAR  14
#define PPI_CH_SW_0         15
#define PPI_CH_SW_1         16

#define PPI_CH_RXEN         PPI_CH_SW_0
#define PPI_CH_TXEN         PPI_CH_SW_1

#define CC_IDX_SW_0  0
#define CC_IDX_SW_1  1

#define CC_IDX_RXEN  CC_IDX_SW_0
#define CC_IDX_TXEN  CC_IDX_SW_1

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

struct latency_tracker {
	uint16_t latency_max;
	uint16_t latency_used;
	uint32_t events_skipped;
	uint32_t events_listened;
};

struct ll_proc_conn_update {
	bool     pending;
	uint16_t instant;
	uint8_t  win_size;
	uint16_t win_offset;
	uint16_t interval;
	uint16_t latency;
	uint16_t timeout;
};

struct ll_proc_chan_map_update {
	bool     pending;
	uint16_t instant;
	uint8_t  chan_map[PDU_CHANNEL_MAP_SIZE];
};

/*===========================================================================
 * Slave-Initiated Procedures
 *===========================================================================*/
struct ll_proc_slave_terminate {
	bool    pending;  /* true = slave 想主动断连 */
	uint8_t reason;   /* 断连原因码 */
};

struct ll_proc_slave_conn_param_req {
	bool     pending;        /* true = 需要发送 CONN_PARAM_REQ */
	bool     sent;           /* true = 已发送, 等待 Master 响应 */
	uint16_t interval_min;
	uint16_t interval_max;
	uint16_t latency;
	uint16_t timeout;
};

/*===========================================================================
 * LE Encryption 状态
 *===========================================================================*/

/*
 * 加密握手阶段 (Slave 视角):
 *
 *   OFF            → 未加密
 *   ENC_RSP_SENT   → 已发送 LL_ENC_RSP, 等待 ack
 *   START_REQ_PEND → LL_ENC_RSP 已被 ack, 需要发送 LL_START_ENC_REQ
 *   WAIT_START_RSP → 已发送 LL_START_ENC_REQ, 等待 Master 的 LL_START_ENC_RSP
 *   ACTIVE         → 完全加密
 */
enum enc_phase {
	ENC_PHASE_OFF = 0,
	ENC_PHASE_RSP_SENT,
	ENC_PHASE_START_REQ_PEND,
	ENC_PHASE_WAIT_START_RSP,
	ENC_PHASE_ACTIVE,
};

struct enc_state {
	enum enc_phase phase;
	bool rx_encrypted;   /* RX 解密开启 */
	bool tx_encrypted;   /* TX 加密开启 */

	/* 密钥材料 */
	uint8_t ltk[16];     /* Long Term Key (本 Demo 硬编码) */
	uint8_t rand[8];     /* from LL_ENC_REQ (用于 HCI 查找 LTK) */
	uint8_t ediv[2];     /* from LL_ENC_REQ */
	uint8_t skd_m[8];    /* from LL_ENC_REQ: SKDm */
	uint8_t iv_m[4];     /* from LL_ENC_REQ: IVm */
	uint8_t skd_s[8];    /* Slave 生成: SKDs */
	uint8_t iv_s[4];     /* Slave 生成: IVs */

	/* 会话密钥 (大端, CCM 直接使用) */
	uint8_t session_key_be[16];

	/* CCM 配置: RX (Master→Slave), TX (Slave→Master) */
	struct ccm ccm_rx;   /* direction=1 */
	struct ccm ccm_tx;   /* direction=0 */
};

/*===========================================================================
 * 全局变量声明
 *===========================================================================*/

extern const uint8_t adv_addr[BDADDR_SIZE];
extern uint8_t adv_data[];
extern uint8_t adv_data_len;
#define ADV_DATA_LEN_MAX 31
extern struct pdu_adv pdu;
extern struct pdu_adv pdu_scan_rsp;
extern struct pdu_adv pdu_rx_buf;

extern struct conn_param conn_params;

extern uint8_t tx_sn;
extern uint8_t rx_nesn;
extern uint16_t conn_event_counter;
extern uint8_t last_unmapped_chan;
extern volatile bool conn_terminated;
extern uint8_t conn_terminate_reason;

extern struct pdu_data tx_pdu_buf;
extern bool tx_pdu_pending;

extern struct pdu_adv pdu_adv_ind;
extern struct pdu_adv pdu_adv_rx;
extern struct pdu_data pdu_data_rx;

extern uint32_t connect_end_rtc;
extern const uint16_t sca_ppm_table[];
extern struct anchor_tracker anchor;
extern struct latency_tracker lat;

extern struct ll_proc_conn_update proc_conn_update;
extern struct ll_proc_chan_map_update proc_chan_map;

extern struct ll_proc_slave_terminate proc_slave_term;
extern struct ll_proc_slave_conn_param_req proc_slave_cpr;
extern struct enc_state enc;

/* ★ 加密缓冲区 (CCM 不允许 in-place, 需要独立输入/输出缓冲区) */
extern uint8_t ccm_scratch_rx[64];
extern uint8_t ccm_scratch_tx[64];

/*===========================================================================
 * SMP 状态
 *===========================================================================*/
enum smp_phase {
	SMP_PHASE_IDLE = 0,
	SMP_PHASE_WAIT_CONFIRM,
	SMP_PHASE_WAIT_RANDOM,
	SMP_PHASE_WAIT_ENC,
	SMP_PHASE_KEY_DIST,
	SMP_PHASE_COMPLETE,
};

struct smp_state_s {
	enum smp_phase phase;
	uint8_t preq[7];      /* Pairing Request (含 code) */
	uint8_t pres[7];      /* Pairing Response (含 code) */
	uint8_t tk[16];       /* Temporary Key (Just Works = 0) */
	uint8_t srand[16];    /* Slave random */
	uint8_t mconfirm[16]; /* Master confirm value */

	/* 密钥分发 */
	uint8_t  dist_ltk[16];
	uint16_t dist_ediv;
	uint8_t  dist_rand[8];
};

extern struct smp_state_s smp_state;

extern volatile uint32_t adv_event_count;
extern volatile uint32_t scan_req_count;
extern volatile uint32_t scan_rsp_count;
extern volatile uint32_t conn_event_rx_ok;
extern volatile uint32_t conn_event_rx_timeout;
extern volatile uint32_t conn_event_rx_crc_err;

/* ★ Multi-PDU 统计 */
extern volatile uint32_t multi_pdu_total_exchanges;
extern volatile uint32_t multi_pdu_events_extended;
extern volatile uint32_t multi_pdu_max_in_event;

extern uint32_t last_rx_aa_us;
extern uint32_t last_rx_ready_us;

#endif /* BLE_COMMON_H */
