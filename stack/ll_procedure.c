/*
 * BLE Slave-Initiated LL Procedures & GATT Notification 调度
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ll_procedure.h"
#include "l2cap.h"

#if BLE_FEATURE_GATT_NOTIFY
#include "att.h"
#include "gatt_db.h"
#endif
#if BLE_FEATURE_ATT
#include "att.h"
#endif
#if BLE_FEATURE_SMP
#include "smp.h"
#endif

/* 触发阈值 (连接事件数) */
static uint32_t cpr_trigger_event;
static uint32_t term_trigger_event;

/* 是否已触发 (防止重复触发) */
static bool cpr_triggered;
static bool term_triggered;

void ll_procedure_init(uint32_t conn_interval_us)
{
	cpr_trigger_event = (SLAVE_CONN_PARAM_REQ_DELAY_S * 1000000UL)
			    / conn_interval_us;
	term_trigger_event = (SLAVE_TERMINATE_DELAY_S * 1000000UL)
			     / conn_interval_us;
	cpr_triggered  = false;
	term_triggered = false;

#if BLE_FEATURE_SLAVE_CPR || BLE_FEATURE_SLAVE_TERMINATE
	printk("[PROC] CPR trigger @ evt#%u, TERM trigger @ evt#%u\n",
	       cpr_trigger_event, term_trigger_event);
#endif
}

void ll_procedure_tick(void)
{
#if BLE_FEATURE_SLAVE_CPR
	if (!cpr_triggered && conn_event_counter >= cpr_trigger_event) {
		cpr_triggered = true;
		l2cap_cpurq.pending      = true;
		l2cap_cpurq.sent         = false;
		l2cap_cpurq.interval_min = 40; /* 50 ms */
		l2cap_cpurq.interval_max = 40;
		l2cap_cpurq.latency      = conn_params.latency;
		l2cap_cpurq.timeout      = conn_params.timeout;
		l2cap_send_conn_param_update_req();
		printk("[CONN] ★ L2CAP CPURQ sent (event #%u)\n",
		       conn_event_counter);
	}
#endif

#if BLE_FEATURE_SLAVE_TERMINATE
	if (!term_triggered && conn_event_counter >= term_trigger_event) {
		term_triggered = true;
		proc_slave_term.pending = true;
		proc_slave_term.reason  = 0x13; /* Remote User Terminated */
		printk("[CONN] ★ Triggering Slave TERMINATE_IND (event #%u)\n",
		       conn_event_counter);
	}
#endif
}

void ll_procedure_gatt_tick(uint32_t conn_interval_us)
{
#if BLE_FEATURE_GATT_NOTIFY
	static uint32_t notify_acc;

	notify_acc++;

	uint32_t events_per_sec = 1000000UL / conn_interval_us;

	if (events_per_sec == 0) {
		events_per_sec = 1;
	}

	if (notify_acc >= events_per_sec) {
		notify_acc = 0;

		uint16_t ctr = sys_get_le16(gatt_counter_val);

		ctr++;
		sys_put_le16(ctr, gatt_counter_val);

		uint16_t ccc = sys_get_le16(gatt_ccc_counter);

		if (ccc & 0x0001) {
			att_notify(HANDLE_COUNTER_VAL, gatt_counter_val, 2);
		}
	}
#else
	(void)conn_interval_us;
#endif
}

void ll_procedure_summary(void)
{
#if BLE_FEATURE_MULTI_PDU
	printk("  --- Multi-PDU ---\n");
	printk("  Total exchanges:    %u\n", multi_pdu_total_exchanges);
	printk("  Events extended:    %u (events with >1 exchange)\n",
	       multi_pdu_events_extended);
	printk("  Max in one event:   %u\n", multi_pdu_max_in_event);
	if (conn_event_counter > 0) {
		printk("  Avg exchanges/evt:  %u.%02u\n",
		       multi_pdu_total_exchanges / conn_event_counter,
		       (multi_pdu_total_exchanges * 100 /
			conn_event_counter) % 100);
	}
#endif

#if BLE_FEATURE_SMP
	printk("  --- Encryption ---\n");
	printk("  Status:             %s\n",
	       enc.phase == ENC_PHASE_ACTIVE ? "ACTIVE" : "NOT ACTIVE");
	printk("  --- SMP ---\n");
	printk("  Status:             %s\n",
	       smp_state.phase == SMP_PHASE_COMPLETE ? "PAIRED" :
	       smp_state.phase == SMP_PHASE_IDLE ? "NOT STARTED" : "IN PROGRESS");
	printk("  TX counter:         %llu\n", enc.ccm_tx.counter);
	printk("  RX counter:         %llu\n", enc.ccm_rx.counter);
#endif

#if BLE_FEATURE_SLAVE_CPR || BLE_FEATURE_SLAVE_TERMINATE
	printk("  --- Slave Procedures ---\n");
#if BLE_FEATURE_SLAVE_CPR
	printk("  CONN_PARAM_REQ:     %s\n",
	       cpr_triggered ? (proc_slave_cpr.sent ? "sent" : "queued")
			     : "not triggered");
#endif
#if BLE_FEATURE_SLAVE_TERMINATE
	printk("  TERMINATE_IND:      %s\n",
	       term_triggered ? "sent" : "not triggered");
#endif
#endif /* BLE_FEATURE_SLAVE_CPR || BLE_FEATURE_SLAVE_TERMINATE */
}

/* ll_procedure_session_init 已拆分:
 * - 基础会话复位 (proc_x/l2cap_cpurq/enc memset + smp_init/att_init/l2cap_init)
 *   上移至 conn_event_reset(), 始终执行, 不依赖 BLE_FEATURE_LL_PROCEDURE。
 * - ll_procedure_init() 在 conn.c 中由 BLE_FEATURE_LL_PROCEDURE 宏保护调用。
 */
