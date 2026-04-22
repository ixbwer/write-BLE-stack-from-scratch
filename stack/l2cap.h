/*
 * BLE PHY L2CAP Signaling Demo — L2CAP 基本帧 + 信令通道
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef L2CAP_H
#define L2CAP_H

#include <stdbool.h>
#include <stdint.h>

struct pdu_data;

/*===========================================================================
 * L2CAP / Upper-protocol adapter boundary
 *
 * 这些定义不再放在 ble_common.h 中，避免所有 LL core 模块默认携带
 * L2CAP/ATT/GATT 相关常量和状态。需要接触上层协议桥接时，显式包含本头。
 *===========================================================================*/

/* Keep this header compatible with the newer shared stack and older
 * demo-local ble_common.h copies during the transition.
 */

/* L2CAP CIDs */
#ifndef L2CAP_CID_ATT
#define L2CAP_CID_ATT       0x0004
#endif
#ifndef L2CAP_CID_SIG
#define L2CAP_CID_SIG       0x0005
#endif
#ifndef L2CAP_CID_SMP
#define L2CAP_CID_SMP       0x0006
#endif

/* L2CAP frame layout */
#ifndef L2CAP_HDR_SIZE
#define L2CAP_HDR_SIZE      4
#endif
#ifndef L2CAP_MAX_PAYLOAD
#define L2CAP_MAX_PAYLOAD   512
#define L2CAP_DEFINE_REASSEMBLY_TYPE 1
#endif
#ifndef LL_DATA_MTU_DEFAULT
#ifdef LL_PAYLOAD_OCTETS_MAX
#define LL_DATA_MTU_DEFAULT LL_PAYLOAD_OCTETS_MAX
#else
#define LL_DATA_MTU_DEFAULT 27
#endif
#endif

/* L2CAP signaling */
#ifndef L2CAP_SIG_MTU
#define L2CAP_SIG_MTU                    23
#endif
#ifndef L2CAP_SIG_CONN_PARAM_UPDATE_REQ
#define L2CAP_SIG_CONN_PARAM_UPDATE_REQ  0x12
#endif
#ifndef L2CAP_SIG_CONN_PARAM_UPDATE_RSP
#define L2CAP_SIG_CONN_PARAM_UPDATE_RSP  0x13
#endif
#ifndef L2CAP_SIG_CMD_REJECT
#define L2CAP_SIG_CMD_REJECT             0x01
#endif

/* L2CAP Connection Parameter Update Request (BLE 4.0 path) */
struct l2cap_cpurq {
	bool     pending;
	bool     sent;
	uint8_t  identifier;     /* 信令 Identifier (1~255, 递增) */
	uint16_t interval_min;   /* 单位: 1.25ms */
	uint16_t interval_max;
	uint16_t latency;
	uint16_t timeout;        /* 单位: 10ms */
};

#ifndef TX_QUEUE_SIZE
#define TX_QUEUE_SIZE  4
#define L2CAP_DEFINE_TX_QUEUE_TYPES 1
#endif

#ifdef L2CAP_DEFINE_REASSEMBLY_TYPE
struct l2cap_reassembly {
	uint8_t  buf[L2CAP_MAX_PAYLOAD + L2CAP_HDR_SIZE];
	uint16_t expected_len;
	uint16_t received_len;
	bool     in_progress;
};
#endif

#ifdef L2CAP_DEFINE_TX_QUEUE_TYPES
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
#endif

extern struct l2cap_cpurq l2cap_cpurq;
extern struct l2cap_reassembly l2cap_reasm;
extern struct tx_queue data_tx_q;
extern uint32_t l2cap_rx_count;
extern uint32_t l2cap_tx_count;

void l2cap_init(void);

bool l2cap_rx_fragment(uint8_t llid, const uint8_t *data, uint8_t len);

void l2cap_process_complete(void);

int l2cap_tx_enqueue(const uint8_t *buf, uint16_t total_len);

bool l2cap_tx_dequeue(struct pdu_data *pdu);

/* L2CAP Connection Parameter Update Request (CID=0x0005, Code=0x12)
 * 返回: true 如果成功入队 */
bool l2cap_send_conn_param_update_req(void);

#endif /* L2CAP_H */
