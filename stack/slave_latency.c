/*
 * BLE Slave Latency: 事件跳过判定与统计
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "slave_latency.h"
#include "hci.h"
#include "l2cap.h"

void slave_latency_reset(uint16_t latency_max)
{
	lat.latency_max  = latency_max;
	lat.latency_used = 0;
	lat.events_skipped  = 0;
	lat.events_listened = 0;
}

bool slave_latency_should_skip(void)
{
	if (lat.latency_used >= lat.latency_max) {
		return false;
	}
	if (data_tx_q.count > 0 || tx_pdu_pending) {
		return false;
	}
	if (proc_conn_update.pending || proc_chan_map.pending) {
		return false;
	}
	if (hci_acl_tx_pending()) {
		return false;
	}

#if BLE_FEATURE_SLAVE_TERMINATE
	if (proc_slave_term.pending) {
		return false;
	}
#endif

#if BLE_FEATURE_SLAVE_CPR
	if (proc_slave_cpr.pending && !proc_slave_cpr.sent) {
		return false;
	}
#endif

#if BLE_FEATURE_SMP
	if (enc.phase != ENC_PHASE_OFF && enc.phase != ENC_PHASE_ACTIVE) {
		return false;
	}
#endif

	return true;
}

void slave_latency_on_skipped(void)
{
	lat.latency_used++;
	lat.events_skipped++;
}

void slave_latency_on_listened(void)
{
	lat.latency_used = 0;
	lat.events_listened++;
}
