/*
 * BLE PHY Slave Latency Demo — 连接态: Slave Latency 跳过连接事件
 *
 * 相比 07_phy_data_channel 的变化:
 *   1. 实现 Slave Latency: Slave 允许跳过最多 connSlaveLatency 个连接事件
 *   2. 跳过时仍然推进锚点 (ticks + remainder 正常累积)
 *   3. WW 在跳过期间按比例增长: ww = periodic × (latency_used + 1)
 *   4. 有数据要发或 latency 到期时必须唤醒
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "conn.h"
#include "hal_radio.h"
#include "ll_pdu.h"
#include "l2cap.h"

/*===========================================================================
 * 判断是否可以跳过当前连接事件
 *
 * BLE Core Spec Vol 6, Part B, 4.5.2:
 *   Slave 可以不监听最多 connSlaveLatency 个连续的连接事件,
 *   但以下情况必须监听:
 *     1. 有数据要发送 (TX 队列非空 / LL Control 待回复)
 *     2. 已经连续跳过 connSlaveLatency 个事件
 *     3. 即将到达 supervision timeout 的安全边界
 *     4. MD 位指示 Master 有更多数据
 *===========================================================================*/
static bool can_skip_event(void)
{
	/* 还没用完 latency 配额 */
	if (lat.latency_used >= lat.latency_max) {
		return false;
	}

	/* 有数据要发 — 必须唤醒 */
	if (data_tx_q.count > 0 || tx_pdu_pending) {
		return false;
	}

	return true;
}

/*===========================================================================
 * 单个连接事件 (与 04 相同)
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
 * 连接态主循环 — Slave Latency 演示
 *
 * ★ Slave Latency 的核心逻辑:
 *
 * 正常连接中, Slave 每个 connInterval 都必须唤醒射频监听 Master。
 * 当 connSlaveLatency > 0 时, Slave 允许"偷懒":
 *   - 最多连续跳过 connSlaveLatency 个连接事件不监听
 *   - 跳过期间锚点仍正常递推, 信道选择仍推进 (chan_sel_1)
 *   - WW 按跳过的总时间累积 (不是单个 interval)
 *   - 一旦有数据要发, 或 latency 配额用完, 必须立刻唤醒
 *
 * 省电效果:
 *   latency=0: 每 30ms 醒一次
 *   latency=4: 最多每 150ms (5×30ms) 才醒一次
 *              射频功耗降为 1/5
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
	l2cap_init();

	lat.latency_max = conn_params.latency;

	radio_configure_conn();
	print_connect_ind();

	printk("[LATENCY] connSlaveLatency = %u (may skip up to %u events)\n",
	       lat.latency_max, lat.latency_max);

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
		/* ★ Slave Latency 判断: 跳过还是监听? */
		bool skip_this_event = false;

		if (lat.latency_max > 0 && can_skip_event()) {
			skip_this_event = true;
			lat.latency_used++;
			lat.events_skipped++;

			/* 跳过时: 仍需推进信道选择 (Core Spec 要求) */
			(void)chan_sel_1();
			conn_event_counter++;
		} else {
			lat.latency_used = 0;
			lat.events_listened++;
		}

		if (!skip_this_event) {
			/* ★ WW 累积: 如果之前跳过了 N 个事件,
			 * WW 需要按 (N+1) × ww_periodic 计算 */
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
			/* ★ 跳过的事件: WW 仍需累积 */
			ww_event_us += ww_periodic_us;
			if (ww_event_us > ww_max_us) {
				ww_event_us = ww_max_us;
			}
		}

		/* 锚点递推 (无论是否跳过, 时间都在前进) */
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
		    anchor.consecutive_misses > 3) {
			printk("[CONN] #%u %s | ww=%u lat_used=%u/%u | "
			       "rx=%u skip=%u listen=%u\n",
			       current_event,
			       skip_this_event ? "SKIP" :
			       (conn_event_rx_ok == current_event ? "OK" : "MISS"),
			       anchor.ww_current_us,
			       lat.latency_used, lat.latency_max,
			       conn_event_rx_ok,
			       lat.events_skipped, lat.events_listened);
		}
	}

	NRF_RADIO->SHORTS = 0;
	NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;

	printk("\n========== Connection Summary ==========\n");
	printk("  Total events:     %u\n", conn_event_counter);
	printk("  Events listened:  %u\n", lat.events_listened);
	printk("  Events skipped:   %u\n", lat.events_skipped);
	printk("  RX OK:            %u\n", conn_event_rx_ok);
	printk("  RX timeout:       %u\n", conn_event_rx_timeout);
	printk("  RX CRC error:     %u\n", conn_event_rx_crc_err);
	printk("  Anchor updates:   %u\n", anchor.anchor_update_count);
	printk("  Max consec miss:  %u\n", anchor.max_consecutive_misses);
	printk("  Slave Latency:    %u (configured)\n", lat.latency_max);
	printk("  Skip ratio:       %u/%u = %u%%\n",
	       lat.events_skipped, conn_event_counter,
	       conn_event_counter > 0 ?
	       (lat.events_skipped * 100) / conn_event_counter : 0);
	printk("=========================================\n");
}
