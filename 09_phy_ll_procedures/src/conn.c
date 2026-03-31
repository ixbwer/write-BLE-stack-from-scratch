/*
 * BLE PHY LL Procedures Demo — 连接态: Instant 机制演示
 *
 * 相比 08_phy_slave_latency 的核心变化:
 *
 *  ┌─────────────────────────────────────────────────────────────────┐
 *  │  Instant 机制:                                                  │
 *  │                                                                 │
 *  │  当 Master 发送 LL_CONNECTION_UPDATE_IND 或 LL_CHANNEL_MAP_IND │
 *  │  时, PDU 中携带一个 instant (uint16_t) 值。                     │
 *  │                                                                 │
 *  │  含义: 新参数在 connEventCounter == instant 时生效。            │
 *  │                                                                 │
 *  │  在 Instant 到达之前, 继续使用旧参数正常通信。                 │
 *  │  在 Instant 到达的那个事件, 切换为新参数。                     │
 *  │                                                                 │
 *  │  如果 Slave 收到 Instant 已经过去, 则视为协议错误,             │
 *  │  应断开连接。                                                   │
 *  │                                                                 │
 *  │  ★ 重要: 当有 pending procedure 且 Instant 还未到达时,        │
 *  │    Slave 不得使用 Slave Latency 跳过事件!                      │
 *  │    必须每个事件都监听, 以确保不会错过 Instant。                │
 *  └─────────────────────────────────────────────────────────────────┘
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "conn.h"
#include "hal_radio.h"
#include "ll_pdu.h"
#include "l2cap.h"

/*===========================================================================
 * Instant 有效性检查
 *
 * BLE Core Spec Vol 6, Part B, 5.1.1:
 *   instant 必须在 (connEventCounter, connEventCounter + 32767) 之间,
 *   即: (instant - connEventCounter) 以 uint16 算术应落在 [1, 32767]。
 *   如果不满足, 说明 Instant 已经过去, 协议错误。
 *===========================================================================*/
static bool instant_passed(uint16_t instant)
{
	uint16_t diff = instant - conn_event_counter;
	/* diff == 0 表示正好是当前事件 — 不算过去 */
	/* diff > 32767 (即高位为1) 表示 instant 在 counter 之前 */
	return (diff > 0x7FFF) && (diff != 0);
}

/*===========================================================================
 * Instant 检查与参数应用
 *
 * 在每个连接事件结束后 (conn_event_counter 已递增) 调用。
 * 如果当前 counter == pending instant, 应用新参数。
 *===========================================================================*/

/*
 * 应用连接更新: 替换 interval, latency, timeout
 * 同时需要重新计算 WW 周期值和 supervision timeout
 *
 * 返回 true 如果本事件触发了更新
 */
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
		/* 检查是否已过期 */
		if (instant_passed(proc_conn_update.instant)) {
			printk("[LL] ERROR: Connection Update instant "
			       "passed! (instant=%u counter=%u)\n",
			       proc_conn_update.instant,
			       conn_event_counter);
			conn_terminated = true;
		}
		return false;
	}

	/* ★ Instant 到达! 应用新参数 */
	printk("[LL] ★ Connection Update applied at event #%u\n",
	       conn_event_counter);
	printk("       interval: %u → %u\n",
	       conn_params.interval, proc_conn_update.interval);
	printk("       latency:  %u → %u\n",
	       conn_params.latency, proc_conn_update.latency);
	printk("       timeout:  %u → %u\n",
	       conn_params.timeout, proc_conn_update.timeout);

	conn_params.win_size   = proc_conn_update.win_size;
	conn_params.win_offset = proc_conn_update.win_offset;
	conn_params.interval   = proc_conn_update.interval;
	conn_params.latency    = proc_conn_update.latency;
	conn_params.timeout    = proc_conn_update.timeout;

	/* 更新 slave latency */
	lat.latency_max = conn_params.latency;
	lat.latency_used = 0;

	/* 重新计算 interval 相关参数 */
	*conn_interval_us = (uint32_t)conn_params.interval * CONN_INT_UNIT_US;
	*interval_ticks = HAL_TICKER_US_TO_TICKS(*conn_interval_us);
	*interval_remainder_ps = HAL_TICKER_REMAINDER(*conn_interval_us);
	*supervision_timeout_us = (uint32_t)conn_params.timeout * 10000;

	anchor.ww_periodic_us = DIV_ROUND_UP(
		combined_sca_ppm * (*conn_interval_us), 1000000);

	uint32_t new_ww_max = ((*conn_interval_us) >> 1) - T_IFS_US;
	anchor.ww_max_us = new_ww_max;

	/* win_size 和 win_offset 用于 Instant 事件的接收窗口 */
	anchor.win_size_event_us =
		(uint32_t)proc_conn_update.win_size * CONN_INT_UNIT_US;

	proc_conn_update.pending = false;
	return true;
}

/*
 * 应用 Channel Map 更新: 替换 chan_map 和 chan_count
 *
 * 返回 true 如果本事件触发了更新
 */
static bool check_apply_chan_map_update(void)
{
	if (!proc_chan_map.pending) {
		return false;
	}

	if (conn_event_counter != proc_chan_map.instant) {
		if (instant_passed(proc_chan_map.instant)) {
			printk("[LL] ERROR: Channel Map instant "
			       "passed! (instant=%u counter=%u)\n",
			       proc_chan_map.instant,
			       conn_event_counter);
			conn_terminated = true;
		}
		return false;
	}

	uint8_t new_count = count_ones(proc_chan_map.chan_map,
				       PDU_CHANNEL_MAP_SIZE);

	printk("[LL] ★ Channel Map Update applied at event #%u\n",
	       conn_event_counter);
	printk("       chan_count: %u → %u\n",
	       conn_params.chan_count, new_count);

	memcpy(conn_params.chan_map, proc_chan_map.chan_map,
	       PDU_CHANNEL_MAP_SIZE);
	conn_params.chan_count = new_count;

	proc_chan_map.pending = false;
	return true;
}

/*===========================================================================
 * Slave Latency 判断 (继承自 05)
 *
 * ★ 新增约束: 当有 pending procedure 时, 不允许跳过事件!
 *   因为 Slave 必须每个事件都监听, 以便在 Instant 到达时
 *   及时应用新参数。
 *===========================================================================*/
static bool can_skip_event(void)
{
	if (lat.latency_used >= lat.latency_max) {
		return false;
	}

	if (data_tx_q.count > 0 || tx_pdu_pending) {
		return false;
	}

	/* ★ 有 pending procedure → 禁止跳过 */
	if (proc_conn_update.pending || proc_chan_map.pending) {
		return false;
	}

	return true;
}

/*===========================================================================
 * 单个连接事件 (与 05 相同)
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
			bool complete = l2cap_rx_fragment(
				pdu_data_rx.ll_id,
				pdu_data_rx.lldata,
				pdu_data_rx.len);
			if (complete) {
				l2cap_process_complete();
			}
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
 * 连接态主循环 — LL Procedures + Instant 演示
 *
 * ★ 与 05 的关键差异:
 *
 *   1. 每个连接事件结束后, 检查是否有 pending procedure 到达 Instant:
 *      - check_apply_conn_update()  → 替换 interval/latency/timeout
 *      - check_apply_chan_map_update() → 替换 channel map
 *
 *   2. 当有 pending procedure 时, Slave Latency 被禁止:
 *      can_skip_event() 在检测到 proc pending 时返回 false
 *
 *   3. Connection Update 生效时, 需要重新计算:
 *      - interval_ticks / interval_remainder_ps (锚点递推步长)
 *      - ww_periodic_us (WW 周期增量)
 *      - ww_max_us (WW 上限)
 *      - supervision_timeout_us
 *      - latency_max
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

	radio_configure_conn();
	print_connect_ind();

	printk("[PROC] Ready for LL Procedures (Instant mechanism)\n");
	printk("[LATENCY] connSlaveLatency = %u\n", lat.latency_max);

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

	uint32_t conn_updates_applied = 0;
	uint32_t chan_map_updates_applied = 0;

	while (!conn_terminated) {
		/* ★ 检查 Instant 到达: 是否需要应用新参数? */
		if (check_apply_conn_update(&conn_interval_us,
					    &interval_ticks,
					    &interval_remainder_ps,
					    &supervision_timeout_us,
					    combined_sca_ppm)) {
			conn_updates_applied++;
			ww_event_us = 0;
			win_size_event_us = anchor.win_size_event_us;
		}

		if (check_apply_chan_map_update()) {
			chan_map_updates_applied++;
		}

		/* Slave Latency 判断 */
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

		/* 锚点递推 */
		next_event_rtc = (next_event_rtc + interval_ticks) &
				 HAL_TICKER_CNTR_MASK;
		next_event_remainder_ps += interval_remainder_ps;
		if (next_event_remainder_ps >= ps_per_tick) {
			next_event_remainder_ps -= ps_per_tick;
			next_event_rtc = (next_event_rtc + 1) &
					 HAL_TICKER_CNTR_MASK;
		}

		uint32_t current_event = conn_event_counter;

		if (current_event <= 10 ||
		    current_event % 50 == 0 ||
		    anchor.consecutive_misses > 3 ||
		    proc_conn_update.pending ||
		    proc_chan_map.pending) {
			printk("[CONN] #%u %s | ww=%u lat=%u/%u | "
			       "pending: cu=%u cm=%u\n",
			       current_event,
			       skip_this_event ? "SKIP" :
			       (conn_event_rx_ok == current_event ? "OK" : "MISS"),
			       anchor.ww_current_us,
			       lat.latency_used, lat.latency_max,
			       proc_conn_update.pending ? 1 : 0,
			       proc_chan_map.pending ? 1 : 0);
		}
	}

	NRF_RADIO->SHORTS = 0;
	NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;

	printk("\n========== Connection Summary ==========\n");
	printk("  Total events:      %u\n", conn_event_counter);
	printk("  Events listened:   %u\n", lat.events_listened);
	printk("  Events skipped:    %u\n", lat.events_skipped);
	printk("  RX OK:             %u\n", conn_event_rx_ok);
	printk("  RX timeout:        %u\n", conn_event_rx_timeout);
	printk("  RX CRC error:      %u\n", conn_event_rx_crc_err);
	printk("  Anchor updates:    %u\n", anchor.anchor_update_count);
	printk("  Max consec miss:   %u\n", anchor.max_consecutive_misses);
	printk("  Conn Updates:      %u\n", conn_updates_applied);
	printk("  Chan Map Updates:  %u\n", chan_map_updates_applied);
	printk("=========================================\n");
}
