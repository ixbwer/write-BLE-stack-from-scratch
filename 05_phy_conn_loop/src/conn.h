/*
 * BLE PHY Connection Loop Demo — 连接态
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONN_H
#define CONN_H

#include "ble_common.h"

/* 连接态主循环: 多事件循环、跳频、SN/NESN 流控、supervision timeout */
void conn_loop(void);

#endif /* CONN_H */
