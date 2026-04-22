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
void radio_configure(void);
void radio_configure_adv(void);
void radio_configure_conn(void);

bool radio_adv_channel_set(uint32_t channel);
void radio_irq_disable_all(void);
void radio_shorts_ready_start_end_disable_set(void);
void radio_shorts_clear(void);
void radio_sw_switch_timer_start(void);
void radio_sw_switch_ppi_disable(void);
bool radio_wait_disabled(uint32_t max_spins, bool force_disable_on_timeout);
bool radio_wait_done(uint32_t max_spins);
bool radio_wait_address(uint32_t max_spins);
bool radio_sw_switch_to_rx(void *rx_pdu, uint32_t wait_spins);
bool radio_sw_switch_to_tx(void *tx_pdu, uint32_t wait_spins);
void radio_sw_switch_cleanup(void);

void ble_sw_switch_reset(void);
void ble_sw_switch(bool tx_to_rx);

void data_chan_set(uint8_t chan);
void send_on_channel(uint32_t channel);

#endif /* HAL_RADIO_INIT_H */
