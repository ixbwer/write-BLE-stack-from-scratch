/*
 * BLE PHY Data Channel Demo — 广播态: ADV_IND 发送与 CONNECT_IND 接收
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ADV_H
#define ADV_H

#include "ble_common.h"

int adv_on_channel(uint32_t channel);
int adv_event(void);

#endif /* ADV_H */
