/*
 * BLE PHY Data Channel Demo — L2CAP 基本帧: 解析、重组与回环
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef L2CAP_H
#define L2CAP_H

#include "ble_common.h"

/* 初始化 L2CAP 重组缓冲区 */
void l2cap_init(void);

/* 处理一个 LL Data PDU 片段
 * llid: PDU_DATA_LLID_DATA_START 或 PDU_DATA_LLID_DATA_CONTINUE
 * data: PDU payload (不含 header)
 * len:  payload 字节数
 * 返回: true 如果一个完整 L2CAP 帧已重组完成 */
bool l2cap_rx_fragment(uint8_t llid, const uint8_t *data, uint8_t len);

/* 对已完成重组的 L2CAP 帧进行处理 (解析 + 回环)
 * 必须在 l2cap_rx_fragment 返回 true 后调用 */
void l2cap_process_complete(void);

/* 将 L2CAP 帧分片放入 TX 队列
 * buf: 完整的 L2CAP 帧 (含 4 字节 header)
 * total_len: 帧总长度 (含 header)
 * 返回: 成功入队的分片数, 0 表示队列满 */
int l2cap_tx_enqueue(const uint8_t *buf, uint16_t total_len);

/* 从 TX 队列取出下一个分片填入 PDU
 * pdu: 目标 PDU 缓冲区
 * 返回: true 如果有数据可发 */
bool l2cap_tx_dequeue(struct pdu_data *pdu);

#endif /* L2CAP_H */
