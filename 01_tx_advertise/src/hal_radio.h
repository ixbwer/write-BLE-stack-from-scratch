/*
 * BLE PHY Raw TX Demo — HAL 层: 硬件初始化与 Radio 配置
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HAL_RADIO_INIT_H
#define HAL_RADIO_INIT_H

#include "ble_common.h"

/* 硬件初始化 */
void hfclk_start(void);
void radio_configure(void);

/* 在指定广播信道上发送一包 */
void send_on_channel(uint32_t channel);

#endif /* HAL_RADIO_INIT_H */
