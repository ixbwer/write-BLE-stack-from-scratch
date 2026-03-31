/*
 * BLE PHY First Anchor Demo — 连接态
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONN_H
#define CONN_H

#include "ble_common.h"

/* 执行首次连接事件: 计算锚点 A0, 在精确时刻 RX 一次, 然后返回 */
void conn_first_event(void);

#endif /* CONN_H */
