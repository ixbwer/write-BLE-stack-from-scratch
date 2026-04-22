/*
 * BLE PHY L2CAP Signaling Demo — L2CAP 基本帧 + 信令通道
 *
 * 相比 11_phy_slave_procedures 的变化:
 *   1. l2cap_process_complete() 重构为 CID-based 派发
 *   2. 新增 CID=0x0005 信令通道处理:
 *      - Connection Parameter Update Request (Code=0x12) 接收
 *      - Connection Parameter Update Response (Code=0x13) 接收
 *      - Command Reject (Code=0x01) 接收
 *   3. 新增 l2cap_send_conn_param_update_req() — Slave 发送 CPURQ
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "l2cap.h"
#include "smp.h"
#include "att.h"

void l2cap_init(void)
{
	memset(&l2cap_reasm, 0, sizeof(l2cap_reasm));
	memset(&data_tx_q, 0, sizeof(data_tx_q));
}

/*===========================================================================
 * 接收: 分片重组
 *===========================================================================*/
bool l2cap_rx_fragment(uint8_t llid, const uint8_t *data, uint8_t len)
{
	if (llid == PDU_DATA_LLID_DATA_START) {
		/* 新 L2CAP 帧的第一个分片 */

		if (len < L2CAP_HDR_SIZE) {
			/* 第一个分片不够放 L2CAP header — 异常, 丢弃 */
			l2cap_reasm.in_progress = false;
			return false;
		}

		/* 提取 L2CAP Length (前 2 字节, 小端) */
		uint16_t l2cap_len = sys_get_le16(data);
		uint16_t total = l2cap_len + L2CAP_HDR_SIZE;

		if (total > sizeof(l2cap_reasm.buf)) {
			printk("[L2CAP] frame too large: %u > %u\n",
			       total, (uint16_t)sizeof(l2cap_reasm.buf));
			l2cap_reasm.in_progress = false;
			return false;
		}

		l2cap_reasm.expected_len = total;
		l2cap_reasm.received_len = 0;
		l2cap_reasm.in_progress = true;

		memcpy(l2cap_reasm.buf, data, len);
		l2cap_reasm.received_len = len;

		if (l2cap_reasm.received_len >= l2cap_reasm.expected_len) {
			l2cap_reasm.in_progress = false;
			return true;
		}
		return false;
	}

	if (llid == PDU_DATA_LLID_DATA_CONTINUE) {
		/* 后续分片 — 追加到重组缓冲区 */

		if (!l2cap_reasm.in_progress) {
			/* 没有匹配的 Start 分片 — 丢弃 */
			return false;
		}

		uint16_t remaining = l2cap_reasm.expected_len -
				     l2cap_reasm.received_len;
		uint16_t copy_len = (len < remaining) ? len : remaining;

		memcpy(l2cap_reasm.buf + l2cap_reasm.received_len,
		       data, copy_len);
		l2cap_reasm.received_len += copy_len;

		if (l2cap_reasm.received_len >= l2cap_reasm.expected_len) {
			l2cap_reasm.in_progress = false;
			return true;
		}
		return false;
	}

	return false;
}

/*===========================================================================
 * L2CAP 信令通道处理 (CID=0x0005)
 *
 * BLE 信令帧格式:
 *   [Code (1)] [Identifier (1)] [Length (2, LE)] [Data...]
 *===========================================================================*/
static void l2cap_sig_handle(const uint8_t *payload, uint16_t len)
{
	if (len < 4) {
		printk("[SIG] frame too short: %u\n", len);
		return;
	}

	uint8_t code = payload[0];
	uint8_t id = payload[1];
	uint16_t cmd_len = sys_get_le16(&payload[2]);

	printk("[SIG] RX: code=0x%02X id=%u len=%u\n", code, id, cmd_len);

	switch (code) {
	case L2CAP_SIG_CONN_PARAM_UPDATE_RSP: {
		/* Master → Slave: 回复我们的 CPURQ */
		if (cmd_len < 2) {
			break;
		}
		uint16_t result = sys_get_le16(&payload[4]);
		printk("[SIG] Connection Parameter Update Response: %s (id=%u)\n",
		       result == 0 ? "ACCEPTED" : "REJECTED", id);

		if (l2cap_cpurq.sent && l2cap_cpurq.identifier == id) {
			l2cap_cpurq.sent = false;
			l2cap_cpurq.pending = false;
		}
		break;
	}

	case L2CAP_SIG_CMD_REJECT: {
		if (cmd_len < 2) {
			break;
		}
		uint16_t reason = sys_get_le16(&payload[4]);
		printk("[SIG] Command Reject: reason=0x%04X\n", reason);
		if (l2cap_cpurq.sent && l2cap_cpurq.identifier == id) {
			l2cap_cpurq.sent = false;
			l2cap_cpurq.pending = false;
		}
		break;
	}

	default:
		printk("[SIG] Unknown signal code 0x%02X\n", code);
		break;
	}
}

/*===========================================================================
 * 处理完整的 L2CAP 帧 — CID-based 派发
 *===========================================================================*/
void l2cap_process_complete(void)
{
	uint16_t l2cap_len = sys_get_le16(l2cap_reasm.buf);
	uint16_t cid = sys_get_le16(l2cap_reasm.buf + 2);
	uint16_t total = l2cap_len + L2CAP_HDR_SIZE;

	l2cap_rx_count++;

	printk("[L2CAP] RX #%u: CID=0x%04X len=%u total=%u |",
	       l2cap_rx_count, cid, l2cap_len, total);
	for (int i = 0; i < total && i < 16; i++) {
		printk(" %02X", l2cap_reasm.buf[i]);
	}
	if (total > 16) {
		printk(" ...");
	}
	printk("\n");

	switch (cid) {
	case L2CAP_CID_SIG:
		l2cap_sig_handle(l2cap_reasm.buf + L2CAP_HDR_SIZE, l2cap_len);
		break;

	case L2CAP_CID_ATT:
#if BLE_FEATURE_ATT
		att_handle(l2cap_reasm.buf + L2CAP_HDR_SIZE, l2cap_len);
#endif
		break;

	case L2CAP_CID_SMP:
#if BLE_FEATURE_SMP
		smp_handle(l2cap_reasm.buf + L2CAP_HDR_SIZE, l2cap_len);
#endif
		break;

	default: {
		/* 其他 CID: 回环 (echo) */
		int queued = l2cap_tx_enqueue(l2cap_reasm.buf, total);
		if (queued > 0) {
			l2cap_tx_count++;
			printk("[L2CAP] TX echo queued: %d fragments\n", queued);
		} else {
			printk("[L2CAP] TX queue full, echo dropped!\n");
		}
		break;
	}
	}
}

/*===========================================================================
 * L2CAP Connection Parameter Update Request (Slave → Master)
 *
 * BLE Core Spec Vol 3, Part A, §4.20
 *
 * L2CAP 帧:
 *   [Len=12 (2)] [CID=0x0005 (2)]
 *   [Code=0x12 (1)] [ID (1)] [CmdLen=8 (2)]
 *   [IntMin (2)] [IntMax (2)] [Latency (2)] [Timeout (2)]
 *===========================================================================*/
bool l2cap_send_conn_param_update_req(void)
{
	uint8_t frame[L2CAP_HDR_SIZE + 12];

	/* L2CAP header */
	sys_put_le16(12, &frame[0]);              /* L2CAP length = 12 */
	sys_put_le16(L2CAP_CID_SIG, &frame[2]);  /* CID = 0x0005 */

	/* Signaling command */
	frame[4] = L2CAP_SIG_CONN_PARAM_UPDATE_REQ; /* Code = 0x12 */
	frame[5] = l2cap_cpurq.identifier;           /* Identifier */
	sys_put_le16(8, &frame[6]);                  /* Data length = 8 */

	sys_put_le16(l2cap_cpurq.interval_min, &frame[8]);
	sys_put_le16(l2cap_cpurq.interval_max, &frame[10]);
	sys_put_le16(l2cap_cpurq.latency, &frame[12]);
	sys_put_le16(l2cap_cpurq.timeout, &frame[14]);

	int queued = l2cap_tx_enqueue(frame, sizeof(frame));
	if (queued > 0) {
		l2cap_cpurq.sent = true;
		printk("[SIG] TX: Connection Parameter Update Request (id=%u)\n",
		       l2cap_cpurq.identifier);
		printk("       interval=[%u,%u] latency=%u timeout=%u\n",
		       l2cap_cpurq.interval_min, l2cap_cpurq.interval_max,
		       l2cap_cpurq.latency, l2cap_cpurq.timeout);
		return true;
	}

	printk("[SIG] TX queue full, CPURQ not sent!\n");
	return false;
}

/*===========================================================================
 * 发送: L2CAP 帧分片入队
 *
 * 将一个完整的 L2CAP 帧拆分为 ≤ LL_DATA_MTU_DEFAULT 字节的 PDU 分片,
 * 放入 TX 队列。conn_event 每次取一个分片发送。
 *===========================================================================*/
int l2cap_tx_enqueue(const uint8_t *buf, uint16_t total_len)
{
	uint16_t offset = 0;
	int frag_count = 0;
	bool first = true;

	while (offset < total_len) {
		if (data_tx_q.count >= TX_QUEUE_SIZE) {
			return 0;  /* 队列满 */
		}

		struct tx_frag *f = &data_tx_q.frags[data_tx_q.tail];
		uint16_t chunk = total_len - offset;

		if (chunk > LL_DATA_MTU_DEFAULT) {
			chunk = LL_DATA_MTU_DEFAULT;
		}

		memcpy(f->data, buf + offset, chunk);
		f->len = (uint8_t)chunk;
		f->llid = first ? PDU_DATA_LLID_DATA_START
			        : PDU_DATA_LLID_DATA_CONTINUE;

		data_tx_q.tail = (data_tx_q.tail + 1) % TX_QUEUE_SIZE;
		data_tx_q.count++;

		offset += chunk;
		frag_count++;
		first = false;
	}

	return frag_count;
}

/*===========================================================================
 * 发送: 从 TX 队列取出一个分片
 *===========================================================================*/
bool l2cap_tx_dequeue(struct pdu_data *pdu)
{
	if (data_tx_q.count == 0) {
		return false;
	}

	struct tx_frag *f = &data_tx_q.frags[data_tx_q.head];

	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = f->llid;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = (data_tx_q.count > 1) ? 1 : 0;
	pdu->len   = f->len;

	memcpy(pdu->lldata, f->data, f->len);

	data_tx_q.head = (data_tx_q.head + 1) % TX_QUEUE_SIZE;
	data_tx_q.count--;

	return true;
}
