/*
 * BLE PHY Data Channel Demo — HAL 层: 硬件初始化与 Radio 配置
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HAL_RADIO_INIT_H
#define HAL_RADIO_INIT_H

#include "ble_common.h"

void hfclk_start(void);
void rtc0_start(void);
void sw_switch_timer_configure(void);
void ppi_configure(void);

void radio_ensure_disabled(void);
void radio_configure_adv(void);
void radio_configure_conn(void);

void data_chan_set(uint8_t chan);

#endif /* HAL_RADIO_INIT_H */
