/*
 * BLE PHY Data Channel Demo — 连接态: 连接事件、数据 PDU 收发与锚点调度
 *
 * 相比 06_phy_anchor 的变化:
 *   1. conn_event() 现在处理 LL Data PDU (LLID=10/01), 不仅仅是 LL Control
 *   2. 接收到的数据分片送入 L2CAP 重组模块
 *   3. 发送队列: 优先发 LL Control 回复, 其次发 L2CAP 回环数据
 *   4. MD 位支持: 如果 TX 队列还有数据, 设 MD=1 请求连续发送
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "conn.h"
#include "hal_radio.h"
#include "ll_pdu.h"
#include "l2cap.h"

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

	/* 等待 RX 完成或 HCTO 超时 */
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

	/*==============================================================
	 * RX→TX 转换 (SW TIFS)
	 *==============================================================*/
	NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN] = 0;
	(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN];
	NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN] = 0;
	(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN];

	/* SN/NESN 流控 */
	if (pdu_data_rx.nesn != tx_sn) {
		tx_sn = pdu_data_rx.nesn;
	}
	bool new_packet = false;
	if (pdu_data_rx.sn == rx_nesn) {
		new_packet = true;
		rx_nesn ^= 1;
	}

	/*==============================================================
	 * ★ 数据 PDU 处理 — 本 Demo 的核心新增功能
	 *
	 * LL Data PDU 的 LLID 字段区分数据类型:
	 *   LLID=11 (0x03): LL Control — 链路层控制包
	 *   LLID=10 (0x02): Data Start — L2CAP 帧的第一个分片
	 *   LLID=01 (0x01): Data Continue — L2CAP 帧的后续分片
	 *   LLID=01 + len=0: Empty PDU — 空包 (ACK only)
	 *==============================================================*/
	tx_pdu_pending = false;

	if (new_packet && pdu_data_rx.len > 0) {
		if (pdu_data_rx.ll_id == PDU_DATA_LLID_CTRL) {
			/* LL Control: 与 03 相同的处理 */
			tx_pdu_pending = handle_ll_control(&pdu_data_rx);
		} else if (pdu_data_rx.ll_id == PDU_DATA_LLID_DATA_START ||
			   pdu_data_rx.ll_id == PDU_DATA_LLID_DATA_CONTINUE) {
			/* ★ L2CAP 数据分片: 送入重组模块 */
			bool complete = l2cap_rx_fragment(
				pdu_data_rx.ll_id,
				pdu_data_rx.lldata,
				pdu_data_rx.len);

			if (complete) {
				/* 完整 L2CAP 帧重组完成 → 处理 (echo) */
				l2cap_process_complete();
			}
		}
	}

	/* ★ 准备 TX 包: 优先级
	 *   1. LL Control 回复 (tx_pdu_pending = true)
	 *   2. L2CAP TX 队列中的数据分片
	 *   3. 空包 (ACK only) */
	if (tx_pdu_pending) {
		tx_pdu_buf.nesn = rx_nesn;
		tx_pdu_buf.sn   = tx_sn;
		radio_pkt_tx_set(&tx_pdu_buf);
	} else if (l2cap_tx_dequeue(&tx_pdu_buf)) {
		/* 从 L2CAP TX 队列取出数据分片 */
		tx_pdu_buf.nesn = rx_nesn;
		tx_pdu_buf.sn   = tx_sn;
		tx_pdu_pending = true;
		radio_pkt_tx_set(&tx_pdu_buf);
	} else {
		build_empty_pdu(&tx_pdu_buf);
		radio_pkt_tx_set(&tx_pdu_buf);
	}

	/* 启用 PPI_CH_TXEN */
	nrf_ppi_channels_enable(NRF_PPI, BIT(PPI_CH_TXEN));

	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;

	/* 等待 TX 完成 */
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
 * 连接态主循环
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
	l2cap_init();

	radio_configure_conn();
	print_connect_ind();

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

	anchor.initial_anchor_us = conn_offset_us;

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
			uint32_t actual_s2a = last_rx_aa_us - last_rx_ready_us;
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
			anchor.ww_current_us = 0;
			anchor.win_size_event_us = 0;
		} else {
			anchor.consecutive_misses++;
			if (anchor.consecutive_misses > anchor.max_consecutive_misses) {
				anchor.max_consecutive_misses = anchor.consecutive_misses;
			}

			uint32_t elapsed_ticks = (next_event_rtc - last_rx_rtc) &
						 HAL_TICKER_CNTR_MASK;
			uint32_t elapsed_us = HAL_TICKER_TICKS_TO_US(elapsed_ticks);

			if (elapsed_us >= supervision_timeout_us) {
				printk("[CONN] Supervision timeout! no RX for %u us\n",
				       elapsed_us);
				break;
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

		if (current_event <= 5 || anchor.consecutive_misses > 3) {
			printk("[CONN] #%u ch=%u %s | ww=%u | rx=%u/%u l2cap_rx=%u tx=%u\n",
			       current_event, last_unmapped_chan,
			       rx_ok ? "OK" : "MISS",
			       anchor.ww_current_us,
			       conn_event_rx_ok, current_event,
			       l2cap_rx_count, l2cap_tx_count);
		}

		/* 延迟打印 L2CAP RX 日志 (在事件结束后, 不影响 TIFS) */
		if (l2cap_last_rx_pending) {
			printk("[L2CAP] RX #%u: CID=0x%04X len=%u",
			       l2cap_rx_count, l2cap_last_rx_cid,
			       l2cap_last_rx_len);
			if (l2cap_last_tx_frags > 0) {
				printk(" → echo %d frags\n",
				       l2cap_last_tx_frags);
			} else {
				printk(" → TX queue full!\n");
			}
			l2cap_last_rx_pending = false;
		}
	}

	NRF_RADIO->SHORTS = 0;
	NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;

	printk("\n========== Connection Summary ==========\n");
	printk("  Total events:   %u\n", conn_event_counter);
	printk("  RX OK:          %u\n", conn_event_rx_ok);
	printk("  RX timeout:     %u\n", conn_event_rx_timeout);
	printk("  RX CRC error:   %u\n", conn_event_rx_crc_err);
	printk("  L2CAP RX:       %u\n", l2cap_rx_count);
	printk("  L2CAP TX echo:  %u\n", l2cap_tx_count);
	printk("  Anchor updates: %u\n", anchor.anchor_update_count);
	printk("  Max consec miss: %u\n", anchor.max_consecutive_misses);
	printk("=========================================\n");
}
