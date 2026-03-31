/*
 * BLE PHY HCI Demo — 连接态: HCI ACL 数据桥接
 *
 * 相比 09_phy_ll_procedures 的核心变化:
 *
 *   1. 空中收到的 L2CAP 数据不再做 echo loopback,
 *      而是通过 hci_send_acl_data() 作为 HCI ACL Data 上报给 Host
 *
 *   2. Host 通过 UART 发来的 HCI ACL Data,
 *      通过 l2cap_tx_enqueue() 送入 TX 队列在空中发出
 *
 *   3. 连接建立时发送 LE Connection Complete Event
 *      连接断开时发送 Disconnection Complete Event
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "conn.h"
#include "hal_radio.h"
#include "ll_pdu.h"
#include "l2cap.h"
#include "hci.h"

/*===========================================================================
 * Instant 检查 (与 06 相同)
 *===========================================================================*/
static bool instant_passed(uint16_t instant)
{
	uint16_t diff = instant - conn_event_counter;
	return (diff > 0x7FFF) && (diff != 0);
}

static bool check_apply_conn_update(uint32_t *conn_interval_us,
				    uint32_t *interval_ticks,
				    uint32_t *interval_remainder_ps,
				    uint32_t *supervision_timeout_us,
				    uint32_t combined_sca_ppm)
{
	if (!proc_conn_update.pending) {
		return false;
	}

	if (conn_event_counter != proc_conn_update.instant) {
		if (instant_passed(proc_conn_update.instant)) {
			printk("[LL] ERROR: Connection Update instant passed!\n");
			conn_terminated = true;
		}
		return false;
	}

	printk("[LL] ★ Connection Update at event #%u\n", conn_event_counter);

	conn_params.win_size   = proc_conn_update.win_size;
	conn_params.win_offset = proc_conn_update.win_offset;
	conn_params.interval   = proc_conn_update.interval;
	conn_params.latency    = proc_conn_update.latency;
	conn_params.timeout    = proc_conn_update.timeout;

	lat.latency_max = conn_params.latency;
	lat.latency_used = 0;

	*conn_interval_us = (uint32_t)conn_params.interval * CONN_INT_UNIT_US;
	*interval_ticks = HAL_TICKER_US_TO_TICKS(*conn_interval_us);
	*interval_remainder_ps = HAL_TICKER_REMAINDER(*conn_interval_us);
	*supervision_timeout_us = (uint32_t)conn_params.timeout * 10000;

	anchor.ww_periodic_us = DIV_ROUND_UP(
		combined_sca_ppm * (*conn_interval_us), 1000000);
	anchor.ww_max_us = ((*conn_interval_us) >> 1) - T_IFS_US;
	anchor.win_size_event_us =
		(uint32_t)proc_conn_update.win_size * CONN_INT_UNIT_US;

	proc_conn_update.pending = false;
	return true;
}

static bool check_apply_chan_map_update(void)
{
	if (!proc_chan_map.pending) {
		return false;
	}

	if (conn_event_counter != proc_chan_map.instant) {
		if (instant_passed(proc_chan_map.instant)) {
			printk("[LL] ERROR: Channel Map instant passed!\n");
			conn_terminated = true;
		}
		return false;
	}

	uint8_t new_count = count_ones(proc_chan_map.chan_map,
				       PDU_CHANNEL_MAP_SIZE);
	printk("[LL] ★ Channel Map Update at event #%u (count=%u)\n",
	       conn_event_counter, new_count);

	memcpy(conn_params.chan_map, proc_chan_map.chan_map,
	       PDU_CHANNEL_MAP_SIZE);
	conn_params.chan_count = new_count;

	proc_chan_map.pending = false;
	return true;
}

static bool can_skip_event(void)
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
	/* ★ HCI ACL 数据等待发送也不能跳过 */
	if (hci_acl_tx_pending()) {
		return false;
	}
	return true;
}

/*===========================================================================
 * 单个连接事件
 *===========================================================================*/
static bool conn_event(uint32_t ticks_at_start, uint32_t remainder_ps,
		       uint32_t hcto_add)
{
	uint32_t ahead = (ticks_at_start - NRF_RTC0->COUNTER) &
			 HAL_TICKER_CNTR_MASK;
	if (ahead > (HAL_TICKER_CNTR_MASK >> 1) ||
	    ahead < RTC0_CMP_OFFSET_MIN) {
		(void)chan_sel_1();
		conn_event_rx_timeout++;
		conn_event_counter++;
		return false;
	}

	uint8_t chan = chan_sel_1();
	data_chan_set(chan);

	NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;
	radio_status_reset();
	radio_tmr_status_reset();

	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;

	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN) | BIT(PPI_CH_TXEN));

	nrf_timer_cc_set(NRF_TIMER1, (nrf_timer_cc_channel_t)CC_IDX_TXEN,
			 T_IFS_US - TX_RAMP_US - RX_CHAIN_DELAY_1M_US);

	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_START);

	radio_pkt_rx_set(&pdu_data_rx);

	nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_STOP);
	NRF_RTC0->EVENTS_COMPARE[2] = 0;

	uint32_t remainder_us = radio_tmr_start(0, ticks_at_start,
						remainder_ps);

	radio_tmr_aa_capture();
	radio_tmr_aa_save(0);

	uint32_t hcto = remainder_us + hcto_add +
			radio_rx_ready_delay_get(0, 0) +
			ADDR_US_1M +
			radio_rx_chain_delay_get(0, 0);
	radio_tmr_hcto_configure(hcto);
	radio_tmr_end_capture();

	{
		uint32_t safety = 0;
		while (NRF_RADIO->EVENTS_DISABLED == 0) {
			if (++safety > 5000000) {
				NRF_RADIO->TASKS_DISABLE = 1;
				while (NRF_RADIO->EVENTS_DISABLED == 0) {
				}
				conn_event_rx_timeout++;
				conn_event_counter++;
				return false;
			}
		}
	}
	NRF_RADIO->EVENTS_DISABLED = 0;

	if (!radio_is_done()) {
		radio_ensure_disabled();
		conn_event_rx_timeout++;
		conn_event_counter++;
		return false;
	}

	if (!radio_crc_is_valid()) {
		radio_ensure_disabled();
		conn_event_rx_crc_err++;
		conn_event_counter++;
		return false;
	}

	conn_event_rx_ok++;

	last_rx_aa_us = radio_tmr_aa_get();
	last_rx_ready_us = radio_tmr_ready_get();

	/* RX→TX */
	NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN] = 0;
	(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN];
	NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN] = 0;
	(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN];

	if (pdu_data_rx.nesn != tx_sn) {
		tx_sn = pdu_data_rx.nesn;
	}
	bool new_packet = false;
	if (pdu_data_rx.sn == rx_nesn) {
		new_packet = true;
		rx_nesn ^= 1;
	}

	tx_pdu_pending = false;
	if (new_packet && pdu_data_rx.len > 0) {
		if (pdu_data_rx.ll_id == PDU_DATA_LLID_CTRL) {
			tx_pdu_pending = handle_ll_control(&pdu_data_rx);
		} else if (pdu_data_rx.ll_id == PDU_DATA_LLID_DATA_START ||
			   pdu_data_rx.ll_id == PDU_DATA_LLID_DATA_CONTINUE) {
			/*
			 * ★ HCI 桥接: 空中收到的数据 → 转发给 Host
			 *
			 * 之前 Demo 是 L2CAP echo loopback,
			 * 现在改为通过 HCI ACL Data 上报给 UART Host。
			 */
			bool is_start = (pdu_data_rx.ll_id ==
					 PDU_DATA_LLID_DATA_START);
			hci_send_acl_data(pdu_data_rx.lldata,
					  pdu_data_rx.len,
					  is_start);
		}
	}

	/* ★ HCI 桥接: Host 发来的 ACL 数据 → 送入 TX 队列以空中发出 */
	if (!tx_pdu_pending && hci_acl_tx_pending()) {
		uint8_t acl_buf[HCI_ACL_MAX_DATA];
		uint16_t acl_len = hci_acl_tx_get(acl_buf, sizeof(acl_buf));
		if (acl_len > 0) {
			l2cap_tx_enqueue(acl_buf, acl_len);
			hci_send_num_completed_pkts(1);
		}
	}

	if (tx_pdu_pending) {
		tx_pdu_buf.nesn = rx_nesn;
		tx_pdu_buf.sn   = tx_sn;
		radio_pkt_tx_set(&tx_pdu_buf);
	} else if (l2cap_tx_dequeue(&tx_pdu_buf)) {
		tx_pdu_buf.nesn = rx_nesn;
		tx_pdu_buf.sn   = tx_sn;
		tx_pdu_pending = true;
		radio_pkt_tx_set(&tx_pdu_buf);
	} else {
		build_empty_pdu(&tx_pdu_buf);
		radio_pkt_tx_set(&tx_pdu_buf);
	}

	nrf_ppi_channels_enable(NRF_PPI, BIT(PPI_CH_TXEN));

	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;

	{
		uint32_t safety = 0;
		while (NRF_RADIO->EVENTS_DISABLED == 0) {
			if (++safety > 2000000) {
				NRF_RADIO->TASKS_DISABLE = 1;
				while (NRF_RADIO->EVENTS_DISABLED == 0) {
				}
				break;
			}
		}
	}
	NRF_RADIO->EVENTS_DISABLED = 0;

	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_TXEN));
	NRF_RADIO->SHORTS = 0;

	if (tx_pdu_pending) {
		tx_sn ^= 1;
	}

	conn_event_counter++;
	return true;
}

/*===========================================================================
 * 连接态主循环 — HCI ACL 桥接
 *===========================================================================*/
void conn_loop(void)
{
	tx_sn = 0;
	rx_nesn = 0;
	conn_event_counter = 0;
	last_unmapped_chan = 0;
	conn_terminated = false;
	tx_pdu_pending = false;
	conn_event_rx_ok = 0;
	conn_event_rx_timeout = 0;
	conn_event_rx_crc_err = 0;

	memset(&anchor, 0, sizeof(anchor));
	memset(&lat, 0, sizeof(lat));
	memset(&proc_conn_update, 0, sizeof(proc_conn_update));
	memset(&proc_chan_map, 0, sizeof(proc_chan_map));
	l2cap_init();

	lat.latency_max = conn_params.latency;
	hci_connected = true;

	radio_configure_conn();
	print_connect_ind();

	/* ★ 向 Host 发送 LE Connection Complete Event */
	hci_send_le_conn_complete();

	uint32_t conn_interval_us = (uint32_t)conn_params.interval * CONN_INT_UNIT_US;
	uint32_t win_size_us = (uint32_t)conn_params.win_size * CONN_INT_UNIT_US;

	uint16_t master_sca_ppm = sca_ppm_table[conn_params.sca & 0x07];
	uint32_t combined_sca_ppm = (uint32_t)master_sca_ppm + LOCAL_SCA_PPM;
	uint32_t ww_periodic_us = DIV_ROUND_UP(
		combined_sca_ppm * conn_interval_us, 1000000);
	uint32_t ww_max_us = (conn_interval_us >> 1) - T_IFS_US;

	anchor.ww_periodic_us = ww_periodic_us;
	anchor.ww_max_us = ww_max_us;

	uint32_t ww_event_us = 0;
	uint32_t win_size_event_us = win_size_us;
	anchor.win_size_event_us = win_size_us;

	uint32_t conn_offset_us = (uint32_t)conn_params.win_offset * CONN_INT_UNIT_US
				+ WIN_DELAY_LEGACY
				- EVENT_TICKER_RES_MARGIN_US
				- EVENT_JITTER_US
				- radio_rx_ready_delay_get(0, 0)
				- ww_periodic_us;

	uint32_t offset_ticks = HAL_TICKER_US_TO_TICKS(conn_offset_us);
	uint32_t offset_remainder_ps = HAL_TICKER_REMAINDER(conn_offset_us);

	uint32_t interval_ticks = HAL_TICKER_US_TO_TICKS(conn_interval_us);
	uint32_t interval_remainder_ps = HAL_TICKER_REMAINDER(conn_interval_us);

	uint32_t ps_per_tick = HAL_TICKER_CNTR_CLK_UNIT_FSEC /
			       HAL_TICKER_FSEC_PER_PSEC;

	uint32_t next_event_rtc = (connect_end_rtc + offset_ticks) &
				  HAL_TICKER_CNTR_MASK;
	uint32_t next_event_remainder_ps = offset_remainder_ps;

	anchor.initial_anchor_rtc = next_event_rtc;
	anchor.current_anchor_rtc = next_event_rtc;
	anchor.last_rx_anchor_rtc = connect_end_rtc;

	uint32_t supervision_timeout_us = (uint32_t)conn_params.timeout * 10000;
	uint32_t last_rx_rtc = connect_end_rtc;

	while (!conn_terminated) {
		/* 处理 HCI 命令 (非阻塞) */
		h4_process();

		/* Instant 检查 */
		(void)check_apply_conn_update(&conn_interval_us,
					      &interval_ticks,
					      &interval_remainder_ps,
					      &supervision_timeout_us,
					      combined_sca_ppm);
		(void)check_apply_chan_map_update();

		/* Slave Latency */
		bool skip_this_event = false;
		if (lat.latency_max > 0 && can_skip_event()) {
			skip_this_event = true;
			lat.latency_used++;
			lat.events_skipped++;
			(void)chan_sel_1();
			conn_event_counter++;
		} else {
			lat.latency_used = 0;
			lat.events_listened++;
		}

		if (!skip_this_event) {
			ww_event_us += ww_periodic_us;
			if (ww_event_us > ww_max_us) {
				ww_event_us = ww_max_us;
			}
			anchor.ww_current_us = ww_event_us;

			uint32_t hcto_add = ((EVENT_JITTER_US +
					      EVENT_TICKER_RES_MARGIN_US +
					      ww_event_us) << 1) +
					    win_size_event_us;

			anchor.current_anchor_rtc = next_event_rtc;

			bool rx_ok = conn_event(next_event_rtc,
						next_event_remainder_ps,
						hcto_add);

			if (rx_ok) {
				uint32_t actual_s2a = last_rx_aa_us -
						      last_rx_ready_us;
				int32_t drift_us = (int32_t)actual_s2a -
					(int32_t)(EVENT_JITTER_US +
						  EVENT_TICKER_RES_MARGIN_US +
						  ADDR_US_1M);

				int64_t adj_ps = (int64_t)next_event_remainder_ps +
						 (int64_t)drift_us * 1000000LL;

				while (adj_ps >= (int64_t)ps_per_tick) {
					adj_ps -= ps_per_tick;
					next_event_rtc = (next_event_rtc + 1) &
							 HAL_TICKER_CNTR_MASK;
				}
				while (adj_ps < 0) {
					adj_ps += ps_per_tick;
					next_event_rtc = (next_event_rtc - 1) &
							 HAL_TICKER_CNTR_MASK;
				}
				next_event_remainder_ps = (uint32_t)adj_ps;

				last_rx_rtc = next_event_rtc;
				anchor.last_rx_anchor_rtc = next_event_rtc;
				anchor.anchor_update_count++;
				anchor.consecutive_misses = 0;

				ww_event_us = 0;
				win_size_event_us = 0;
			} else {
				anchor.consecutive_misses++;
				if (anchor.consecutive_misses >
				    anchor.max_consecutive_misses) {
					anchor.max_consecutive_misses =
						anchor.consecutive_misses;
				}

				uint32_t elapsed_ticks =
					(next_event_rtc - last_rx_rtc) &
					HAL_TICKER_CNTR_MASK;
				uint32_t elapsed_us =
					HAL_TICKER_TICKS_TO_US(elapsed_ticks);

				if (elapsed_us >= supervision_timeout_us) {
					printk("[CONN] Supervision timeout!\n");
					break;
				}
			}
		} else {
			ww_event_us += ww_periodic_us;
			if (ww_event_us > ww_max_us) {
				ww_event_us = ww_max_us;
			}
		}

		next_event_rtc = (next_event_rtc + interval_ticks) &
				 HAL_TICKER_CNTR_MASK;
		next_event_remainder_ps += interval_remainder_ps;
		if (next_event_remainder_ps >= ps_per_tick) {
			next_event_remainder_ps -= ps_per_tick;
			next_event_rtc = (next_event_rtc + 1) &
					 HAL_TICKER_CNTR_MASK;
		}
	}

	NRF_RADIO->SHORTS = 0;
	NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;

	/* ★ 向 Host 发送 Disconnection Complete Event */
	hci_connected = false;
	hci_send_disconnect_complete(0x13); /* Remote User Terminated */

	printk("\n========== Connection Summary (HCI) ==========\n");
	printk("  Total events:     %u\n", conn_event_counter);
	printk("  RX OK:            %u\n", conn_event_rx_ok);
	printk("  RX timeout:       %u\n", conn_event_rx_timeout);
	printk("  RX CRC error:     %u\n", conn_event_rx_crc_err);
	printk("================================================\n");
}
