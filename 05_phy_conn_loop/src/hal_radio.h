/*
 * BLE PHY Connection Loop Demo — HAL 层: 硬件初始化与 Radio 配置
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HAL_RADIO_INIT_H
#define HAL_RADIO_INIT_H

#include "ble_common.h"

/* 硬件初始化 */
void hfclk_start(void);
void rtc0_start(void);
void sw_switch_timer_configure(void);
void ppi_configure(void);

/* Radio 状态管理 */
void radio_ensure_disabled(void);
void radio_configure_adv(void);
void radio_configure_conn(void);

/* 数据信道频率设置 */
void data_chan_set(uint8_t chan);

#endif /* HAL_RADIO_INIT_H */
