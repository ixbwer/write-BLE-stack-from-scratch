/*
 * BLE PHY Passive Scan Demo — 广播信道被动扫描模块
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BLE_PASSIVE_SCAN_H
#define BLE_PASSIVE_SCAN_H

#include "ble_common.h"

struct passive_scan_stats {
	uint32_t windows;
	uint32_t packets_ok;
	uint32_t rx_timeouts;
	uint32_t crc_errors;
	uint32_t ch37_hits;
	uint32_t ch38_hits;
	uint32_t ch39_hits;
	uint32_t adv_ind;
	uint32_t direct_ind;
	uint32_t nonconn_ind;
	uint32_t scan_req;
	uint32_t scan_rsp;
	uint32_t connect_ind;
	uint32_t scan_ind;
	uint32_t ext_ind;
	uint32_t unknown;
};

void passive_scan_init(void);
bool passive_scan_listen_channel(uint8_t channel);
void passive_scan_cycle(void);
const struct passive_scan_stats *passive_scan_stats_get(void);

#endif /* BLE_PASSIVE_SCAN_H */