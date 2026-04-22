/*
 * BLE 连接锚点: 窗口扩展、漂移修正、Supervision Timeout 辅助
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "anchor.h"
#include "ll_pdu.h"
#include "hal_radio.h"

/* ─── 内部辅助: Instant 是否已过 ─── */
static bool instant_passed(uint16_t instant)
{
	uint16_t diff = instant - conn_event_counter;

	return (diff > 0x7FFF) && (diff != 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * LL Procedure Instant 检查
 * ═══════════════════════════════════════════════════════════════════════════ */

bool anchor_apply_conn_update(uint32_t *conn_interval_us,
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
			conn_terminate_reason = 0x28;
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

	lat.latency_max  = conn_params.latency;
	lat.latency_used = 0;

	*conn_interval_us      = (uint32_t)conn_params.interval * CONN_INT_UNIT_US;
	*interval_ticks        = HAL_TICKER_US_TO_TICKS(*conn_interval_us);
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

bool anchor_apply_chan_map(void)
{
	if (!proc_chan_map.pending) {
		return false;
	}

	if (conn_event_counter != proc_chan_map.instant) {
		if (instant_passed(proc_chan_map.instant)) {
			printk("[LL] ERROR: Channel Map instant passed!\n");
			conn_terminate_reason = 0x28;
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

/* ═══════════════════════════════════════════════════════════════════════════
 * 锚点漂移修正
 * ═══════════════════════════════════════════════════════════════════════════ */

void anchor_drift_correct(uint32_t *next_event_rtc,
			  uint32_t *next_event_remainder_ps,
			  uint32_t ps_per_tick)
{
	/*
	 * actual_s2a: RX access-address 捕获时刻 — RADIO READY 时刻
	 * expected:   EVENT_JITTER + TICKER_MARGIN + ADDR_US_1M (标称值)
	 * drift_us:   正值 = Master 晚到; 负值 = Master 早到
	 */
	uint32_t actual_s2a = last_rx_aa_us - last_rx_ready_us;
	int32_t  drift_us   = (int32_t)actual_s2a -
			      (int32_t)(EVENT_JITTER_US +
					EVENT_TICKER_RES_MARGIN_US +
					ADDR_US_1M);

	int64_t adj_ps = (int64_t)(*next_event_remainder_ps) +
			 (int64_t)drift_us * 1000000LL;

	while (adj_ps >= (int64_t)ps_per_tick) {
		adj_ps -= ps_per_tick;
		*next_event_rtc = (*next_event_rtc + 1) & HAL_TICKER_CNTR_MASK;
	}
	while (adj_ps < 0) {
		adj_ps += ps_per_tick;
		*next_event_rtc = (*next_event_rtc - 1) & HAL_TICKER_CNTR_MASK;
	}
	*next_event_remainder_ps = (uint32_t)adj_ps;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Supervision Timeout 检查
 * ═══════════════════════════════════════════════════════════════════════════ */

bool anchor_supervision_check(uint32_t next_event_rtc,
			      uint32_t last_rx_rtc,
			      uint32_t supervision_timeout_us)
{
	uint32_t elapsed_ticks = (next_event_rtc - last_rx_rtc) &
				 HAL_TICKER_CNTR_MASK;
	uint32_t elapsed_us    = HAL_TICKER_TICKS_TO_US(elapsed_ticks);

	if (elapsed_us >= supervision_timeout_us) {
		printk("[CONN] Supervision timeout!\n");
		conn_terminate_reason = 0x08;
		conn_terminated = true;
		return true;
	}
	return false;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 锚点推进
 * ═══════════════════════════════════════════════════════════════════════════ */

void anchor_catchup(uint32_t *next_event_rtc,
		    uint32_t *next_event_remainder_ps,
		    uint32_t interval_ticks,
		    uint32_t interval_remainder_ps,
		    uint32_t ps_per_tick,
		    uint32_t *ww_event_us)
{
	/*
	 * HCI 事件发送、printk 等启动延迟可能让时间过了第一个锚点。
	 * 跳过所有过时事件, 保持 chan_sel_1() + conn_event_counter 与 Master 同步。
	 */
	uint32_t min_ahead = RTC0_CMP_OFFSET_MIN +
			     HAL_TICKER_US_TO_TICKS(200);

	while (1) {
		uint32_t now   = NRF_RTC0->COUNTER & HAL_TICKER_CNTR_MASK;
		uint32_t ahead = (*next_event_rtc - now) & HAL_TICKER_CNTR_MASK;

		if (ahead < (HAL_TICKER_CNTR_MASK >> 1) && ahead >= min_ahead) {
			break;
		}

		(void)chan_sel_1();
		conn_event_counter++;

		*next_event_rtc = (*next_event_rtc + interval_ticks) &
				  HAL_TICKER_CNTR_MASK;
		*next_event_remainder_ps += interval_remainder_ps;
		if (*next_event_remainder_ps >= ps_per_tick) {
			*next_event_remainder_ps -= ps_per_tick;
			*next_event_rtc = (*next_event_rtc + 1) &
					  HAL_TICKER_CNTR_MASK;
		}

		*ww_event_us += anchor.ww_periodic_us;
		if (*ww_event_us > anchor.ww_max_us) {
			*ww_event_us = anchor.ww_max_us;
		}
	}
}

void anchor_advance(uint32_t *next_event_rtc,
		    uint32_t *next_event_remainder_ps,
		    uint32_t interval_ticks,
		    uint32_t interval_remainder_ps,
		    uint32_t ps_per_tick)
{
	*next_event_rtc = (*next_event_rtc + interval_ticks) &
			  HAL_TICKER_CNTR_MASK;
	*next_event_remainder_ps += interval_remainder_ps;
	if (*next_event_remainder_ps >= ps_per_tick) {
		*next_event_remainder_ps -= ps_per_tick;
		*next_event_rtc = (*next_event_rtc + 1) &
				  HAL_TICKER_CNTR_MASK;
	}
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 会话级锚点管理 (供 conn_loop() 直接使用的高层接口)
 * ═══════════════════════════════════════════════════════════════════════════ */

void anchor_session_init(struct anchor_session *s)
{
	memset(&anchor, 0, sizeof(anchor));

	uint32_t conn_interval_us = (uint32_t)conn_params.interval * CONN_INT_UNIT_US;
	uint16_t master_sca_ppm   = sca_ppm_table[conn_params.sca & 0x07];
	uint32_t combined_sca_ppm = (uint32_t)master_sca_ppm + LOCAL_SCA_PPM;

	anchor.ww_periodic_us    = DIV_ROUND_UP(combined_sca_ppm * conn_interval_us,
						 1000000);
	anchor.ww_max_us         = (conn_interval_us >> 1) - T_IFS_US;
	anchor.win_size_event_us = (uint32_t)conn_params.win_size * CONN_INT_UNIT_US;

	uint32_t conn_offset_us =
		(uint32_t)conn_params.win_offset * CONN_INT_UNIT_US
		+ WIN_DELAY_LEGACY
		- EVENT_TICKER_RES_MARGIN_US
		- EVENT_JITTER_US
		- radio_rx_ready_delay_get(0, 0)
		- anchor.ww_periodic_us;

	s->conn_interval_us       = conn_interval_us;
	s->interval_ticks         = HAL_TICKER_US_TO_TICKS(conn_interval_us);
	s->interval_remainder_ps  = HAL_TICKER_REMAINDER(conn_interval_us);
	s->supervision_timeout_us = (uint32_t)conn_params.timeout * 10000;
	s->combined_sca_ppm       = combined_sca_ppm;
	s->ps_per_tick            = HAL_TICKER_CNTR_CLK_UNIT_FSEC /
				    HAL_TICKER_FSEC_PER_PSEC;

	s->next_event_rtc =
		(connect_end_rtc + HAL_TICKER_US_TO_TICKS(conn_offset_us)) &
		HAL_TICKER_CNTR_MASK;
	s->next_event_remainder_ps = HAL_TICKER_REMAINDER(conn_offset_us);

	s->ww_event_us       = 0;
	s->win_size_event_us = anchor.win_size_event_us;
	s->last_rx_rtc       = connect_end_rtc;

	anchor.initial_anchor_rtc = s->next_event_rtc;
	anchor.current_anchor_rtc = s->next_event_rtc;
	anchor.last_rx_anchor_rtc = connect_end_rtc;

	/* 跳过启动延迟造成的过时锚点 */
	anchor_catchup(&s->next_event_rtc, &s->next_event_remainder_ps,
		       s->interval_ticks, s->interval_remainder_ps,
		       s->ps_per_tick, &s->ww_event_us);
}

static void anchor_accum_ww(struct anchor_session *s)
{
	s->ww_event_us += anchor.ww_periodic_us;
	if (s->ww_event_us > anchor.ww_max_us) {
		s->ww_event_us = anchor.ww_max_us;
	}
}

uint32_t anchor_prepare_event(struct anchor_session *s)
{
	anchor_accum_ww(s);
	anchor.ww_current_us      = s->ww_event_us;
	anchor.current_anchor_rtc = s->next_event_rtc;

	return ((EVENT_JITTER_US + EVENT_TICKER_RES_MARGIN_US +
		 s->ww_event_us) << 1) + s->win_size_event_us;
}

void anchor_skip_event(struct anchor_session *s)
{
	anchor_accum_ww(s);
	(void)chan_sel_1();
	conn_event_counter++;
}

void anchor_on_rx_ok(struct anchor_session *s)
{
	anchor_drift_correct(&s->next_event_rtc, &s->next_event_remainder_ps,
			     s->ps_per_tick);

	s->last_rx_rtc            = s->next_event_rtc;
	anchor.last_rx_anchor_rtc = s->next_event_rtc;
	anchor.anchor_update_count++;
	anchor.consecutive_misses = 0;
	s->ww_event_us            = 0;
	s->win_size_event_us      = 0;
}

bool anchor_on_rx_miss(struct anchor_session *s)
{
	anchor.consecutive_misses++;
	if (anchor.consecutive_misses > anchor.max_consecutive_misses) {
		anchor.max_consecutive_misses = anchor.consecutive_misses;
	}
	return anchor_supervision_check(s->next_event_rtc, s->last_rx_rtc,
					s->supervision_timeout_us);
}
