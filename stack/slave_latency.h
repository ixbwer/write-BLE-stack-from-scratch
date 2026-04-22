/*
 * BLE Slave Latency: 事件跳过判定与统计
 *
 * 本模块在 07_phy_slave_latency 引入。
 *
 * Slave Latency 允许 Slave 在无数据时跳过最多 latency 个连接事件,
 * 减少功耗。conn_loop() 每个事件开头调用 slave_latency_should_skip()
 * 判断本事件是否可以跳过。
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SLAVE_LATENCY_H
#define SLAVE_LATENCY_H

#include "ble_common.h"

/* 重置 latency 计数器 (conn_loop 开头调用) */
void slave_latency_reset(uint16_t latency_max);

/*
 * 判断本连接事件是否可以跳过。
 *
 * 可跳过的条件 (全部满足):
 *   - latency_max > 0 且已用次数 < latency_max
 *   - TX 队列为空 (data_tx_q.count == 0 && !tx_pdu_pending)
 *   - 没有待处理的 Instant 过程 (conn_update / chan_map)
 *   - HCI ACL 无待发数据
 *   - Slave 无主动过程待发 (terminate / CPR)
 *   - 加密握手未进行中
 */
bool slave_latency_should_skip(void);

/* 本事件被跳过: 更新 latency_used / events_skipped */
void slave_latency_on_skipped(void);

/* 本事件被执行: 重置 latency_used, 更新 events_listened */
void slave_latency_on_listened(void);

#endif /* SLAVE_LATENCY_H */
