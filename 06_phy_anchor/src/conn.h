/*
 * BLE PHY Anchor Demo — 连接态: 连接事件与锚点调度
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONN_H
#define CONN_H

#include "ble_common.h"

/* 连接态主循环: 锚点计算、递推、更新的完整演示 */
void conn_loop(void);

#endif /* CONN_H */
