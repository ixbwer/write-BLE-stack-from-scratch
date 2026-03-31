/*
 * BLE PHY CONNECT_IND Demo — 广播态
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ADV_H
#define ADV_H

#include "ble_common.h"

/* 在单个广播信道上发送 ADV_IND 并等待 CONNECT_IND
 * 返回: 1 = 收到有效 CONNECT_IND, 0 = 未收到 */
int adv_on_channel(uint32_t channel);

/* 在 37/38/39 三个信道上轮流广播
 * 返回: 1 = 连接建立, 0 = 本轮无连接 */
int adv_event(void);

#endif /* ADV_H */
