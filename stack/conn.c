/*
 * BLE 连接态主编排层
 *
 * conn_loop() 是连接阶段的顶层驱动, 只做模块协调, 具体实现细节全部
 * 下沉到各子模块:
 *
 *   anchor.c      — 锚点递推、WW 窗口扩展、漂移修正、Supervision Timeout
 *   conn_event.c  — 单次连接事件 RX/TX 循环, 连接事件计数、RADIO/PPI 配置
 *   slave_latency.c — 事件跳过判定 (07+)
 *   ll_procedure.c  — Slave 主动过程、上层栈 (SMP/ATT/L2CAP) 初始化、GATT notify
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "conn.h"
#include "anchor.h"
#include "conn_event.h"
#if BLE_FEATURE_SLAVE_LATENCY
#include "slave_latency.h"
#endif
#if BLE_FEATURE_LL_PROCEDURE
#include "ll_procedure.h"
#endif
#include "hci.h"

/*===========================================================================
 * 连接态主循环
 *===========================================================================*/

void conn_loop(void)
{
	struct anchor_session s;
	bool first_rx_seen = false;

	conn_terminated       = false;
	conn_terminate_reason = 0x13;

	/* conn_event_reset() 内部已包含 proc_x/l2cap_cpurq/enc 等基础复位 */
	conn_event_reset();
#if BLE_FEATURE_LL_PROCEDURE
	ll_procedure_init((uint32_t)conn_params.interval * CONN_INT_UNIT_US);
#endif
	anchor_session_init(&s);
#if BLE_FEATURE_SLAVE_LATENCY
	slave_latency_reset(conn_params.latency);
#endif

	hci_connected = true;

	while (!conn_terminated) {
		h4_process();

		anchor_apply_conn_update(&s.conn_interval_us,
					 &s.interval_ticks,
					 &s.interval_remainder_ps,
					 &s.supervision_timeout_us,
					 s.combined_sca_ppm);
		anchor_apply_chan_map();
#if BLE_FEATURE_LL_PROCEDURE
		ll_procedure_tick();
#endif

#if BLE_FEATURE_SLAVE_LATENCY
		if (slave_latency_should_skip()) {
			slave_latency_on_skipped();
			anchor_skip_event(&s);
		} else {
			slave_latency_on_listened();
#else
		{
#endif
			uint32_t hcto_add = anchor_prepare_event(&s);

			bool rx_ok = conn_event_run(s.next_event_rtc,
						    s.next_event_remainder_ps,
						    hcto_add);

			if (rx_ok) {
				if (!first_rx_seen) {
					first_rx_seen = true;
					hci_send_le_conn_complete();
					printk("[CONN] Connected evt#%u\n",
					       conn_event_counter);
				}
				anchor_on_rx_ok(&s);
			} else if (anchor_on_rx_miss(&s)) {
				break;
			}
		}

		anchor_advance(&s.next_event_rtc, &s.next_event_remainder_ps,
			       s.interval_ticks, s.interval_remainder_ps,
			       s.ps_per_tick);
#if BLE_FEATURE_LL_PROCEDURE
		ll_procedure_gatt_tick(s.conn_interval_us);
#endif
	}

	conn_event_teardown();
	hci_connected = false;
	hci_send_disconnect_complete(conn_terminate_reason);

	conn_event_summary();
#if BLE_FEATURE_LL_PROCEDURE
	ll_procedure_summary();
#endif
	printk("====================================================\n");
}
