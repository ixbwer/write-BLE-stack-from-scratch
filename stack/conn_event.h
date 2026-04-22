/*
 * 单次 BLE 连接事件: 锚点 RX → RX/TX 交互循环
 *
 * 本模块在 04_phy_connection 引入, 封装一次连接事件内的完整射频操作:
 *   1. 在指定锚点时刻开启 RADIO RX (首次 RX)
 *   2. 处理收到的 PDU (ARQ、LL Control、HCI ACL 桥接)
 *   3. 构造并发送 TX PDU
 *   4. 若双方 MD=1 则在同一事件内继续 RX→TX 循环 (Multi-PDU, 10+)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONN_EVENT_H
#define CONN_EVENT_H

#include "ble_common.h"

/*
 * conn_event_reset — 连接进入时初始化
 *
 * 清零所有连接事件计数器与 ARQ 状态 (tx_sn, rx_nesn, conn_event_counter,
 * last_unmapped_chan, conn_event_rx_*, multi_pdu_*, tx_pdu_pending),
 * 调用 radio_configure_conn() 切换 RADIO 到连接态模板, 并重新绑定
 * PPI_CH_RXEN / PPI_CH_TXEN (adv.c 可能已改过这两个通道的端点)。
 */
void conn_event_reset(void);

/*
 * conn_event_teardown — 连接结束时清理
 *
 * 关闭 RADIO SHORTS / 禁用所有中断, 回到空闲状态。
 */
void conn_event_teardown(void);

/*
 * conn_event_summary — 打印连接事件统计摘要
 *
 * 输出 Total events / RX OK / RX timeout / RX CRC error / Disconnect reason
 * 等计数器。Multi-PDU 相关统计由 conn_event.c 内部输出 (如已启用)。
 */
void conn_event_summary(void);

/*
 * conn_event_run — 执行一次连接事件
 *
 *   ticks_at_start  — RTC0 tick, RADIO RXEN 应在此时刻触发
 *   remainder_ps    — 亚 tick 余量 (picoseconds), 用于亚 tick 精度补偿
 *   hcto_add        — 在标称 HCTO 之上额外增加的微秒数
 *                     (= 2×(WW + JITTER + TICKER_MARGIN) + win_size)
 *
 * 返回 true  — 至少收到一个 CRC 正确的包 (锚点命中)
 *       false — RX 超时或 CRC 错误 (事件丢失)
 */
bool conn_event_run(uint32_t ticks_at_start, uint32_t remainder_ps,
		    uint32_t hcto_add);

#endif /* CONN_EVENT_H */
