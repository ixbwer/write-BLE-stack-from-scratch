/*
 * BLE PHY Data Channel Demo — L2CAP 基本帧: 解析、重组与回环
 *
 * BLE L2CAP Basic Frame 格式:
 *   [Length (2 bytes, LE)] [Channel ID (2 bytes, LE)] [Payload...]
 *
 * 空口传输时, 一个 L2CAP 帧可能被拆分成多个 LL Data PDU:
 *   - 第一个分片: LLID = 10 (Start), 携带 L2CAP header + 前 N 字节
 *   - 后续分片:   LLID = 01 (Continuation), 携带剩余字节
 *
 * 本模块实现:
 *   1. 接收方向: 按 LLID 重组分片为完整 L2CAP 帧
 *   2. 发送方向: 将完整 L2CAP 帧拆分为 ≤27 字节的 LL Data PDU
 *   3. 回环逻辑: 收到的数据原样发回 (echo)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "l2cap.h"

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
 * 处理完整的 L2CAP 帧
 *===========================================================================*/
void l2cap_process_complete(void)
{
	uint16_t l2cap_len = sys_get_le16(l2cap_reasm.buf);
	uint16_t cid = sys_get_le16(l2cap_reasm.buf + 2);
	uint16_t total = l2cap_len + L2CAP_HDR_SIZE;

	l2cap_rx_count++;

	/*
	 * ★ 不在此处 printk！
	 *
	 * 本函数在连接事件的 RX 处理中被调用，处于 SW TIFS 的关键路径上:
	 * RX END → 处理数据 → 准备 TX → PPI 触发 TXEN (窗口仅 ~100µs)
	 *
	 * printk 通过 UART 输出 1 行 hex dump 约需 5-10ms，远超 TIFS 窗口，
	 * 会导致 TX PPI 错过触发时机，echo 的第一个分片 (LLID=Start) 丢失。
	 *
	 * 解决方案: 保存最近一次 RX 信息，conn_loop 中在事件结束后延迟打印。
	 */
	l2cap_last_rx_cid = cid;
	l2cap_last_rx_len = l2cap_len;
	l2cap_last_rx_pending = true;

	/* 回环 (echo): 将收到的完整 L2CAP 帧原样发回 */
	int queued = l2cap_tx_enqueue(l2cap_reasm.buf, total);

	if (queued > 0) {
		l2cap_tx_count++;
		l2cap_last_tx_frags = queued;
	} else {
		l2cap_last_tx_frags = 0;
	}
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
