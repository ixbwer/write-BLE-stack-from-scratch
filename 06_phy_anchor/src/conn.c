/*
 * BLE PHY Anchor Demo — 连接态: 连接事件与锚点调度
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "conn.h"
#include "hal_radio.h"
#include "ll_pdu.h"

/*===========================================================================
 * 单个连接事件 (使用 HAL 硬件定时 + SW TIFS)
 *
 * ★ 锚点相关的核心操作:
 *
 *   radio_tmr_start(0, ticks_at_start, remainder_ps)
 *     → RTC0 CC + PPI + TIMER0 在精确的锚点时刻启动 Radio RX
 *
 * ★ SW TIFS RX→TX 转换:
 *
 *   RX END → PPI 清零 Timer1
 *   Timer1 CC[1]=110μs → PPI 触发 TXEN
 *   TXEN → 40μs ramp → TX START (T_IFS=150μs)
 *===========================================================================*/
static bool conn_event(uint32_t ticks_at_start, uint32_t remainder_ps,
		       uint32_t hcto_add)
{
	/* 检查目标 RTC tick 是否在未来 */
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

	/* ★ 清理上一个事件遗留的 HAL PPI/Timer 状态
	 *   radio_tmr_status_reset() 禁用所有 HAL radio PPI 通道 (CH6/7/20-27),
	 *   并禁用 RTC0 CC[2] 事件产生。
	 *   这防止遗留的 HCTO 或 RXEN/TXEN PPI 干扰新事件。 */
	radio_tmr_status_reset();

	/* SW TIFS: 不使用 DISABLED_TXEN SHORT, RX→TX 由 Timer1+PPI 驱动 */
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;

	/* 确保 PPI 切换通道禁用 */
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN) | BIT(PPI_CH_TXEN));

	/* 设置 Timer1 CC[1] for RX→TX 转换
	 * CC = T_IFS - TX_RAMP - RX_CHAIN_DELAY = 150 - 40 - 10 = 100μs
	 * 补偿 RX chain delay: EVENTS_END 比实际 air-time 结束晚 ~9.4μs,
	 * 所以 CC 值需要减去这个延迟以保证 T_IFS=150±2μs */
	nrf_timer_cc_set(NRF_TIMER1, (nrf_timer_cc_channel_t)CC_IDX_TXEN,
			 T_IFS_US - TX_RAMP_US - RX_CHAIN_DELAY_1M_US);

	/* 启动 Timer1 */
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_START);

	radio_pkt_rx_set(&pdu_data_rx);

	/* ★ 关键: 每次调用 radio_tmr_start 前必须确保硬件状态干净
	 *
	 *   1. 停止 TIMER0: radio_tmr_start 只做 CLEAR 不做 STOP,
	 *      如果 TIMER0 仍在运行, CLEAR 后计数器从 0 重新开始,
	 *      CC[0]=remainder_us 会立即触发 RXEN (不等 RTC 锚点)
	 *
	 *   2. 清除 RTC0 EVENTS_COMPARE[2]: PPI 是边沿触发 (0→1),
	 *      上一个事件留下的锁存值 =1 会阻止新的比较匹配触发 PPI,
	 *      导致 TIMER0 永远不会被 START → RXEN/HCTO 都不会触发
	 *      → while(EVENTS_DISABLED==0) 死循环 */
	nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_STOP);
	NRF_RTC0->EVENTS_COMPARE[2] = 0;

	/* ★ 这里将锚点 tick 传给硬件: radio_tmr_start 内部配置
	 *    RTC0 CC + PPI + TIMER0, 在精确的锚点时刻启动 Radio RX */
	uint32_t remainder_us = radio_tmr_start(0, ticks_at_start,
						remainder_ps);

	radio_tmr_aa_capture();
	radio_tmr_aa_save(0);

	/* HCTO: Header-Complete TimeOut
	 * 保护机制: 如果在锚点 + HCTO 时间内没收到包头, 自动 DISABLE Radio */
	uint32_t hcto = remainder_us + hcto_add +
			radio_rx_ready_delay_get(0, 0) +
			ADDR_US_1M +
			radio_rx_chain_delay_get(0, 0);
	radio_tmr_hcto_configure(hcto);

	radio_tmr_end_capture();

	/* 等待: 要么收到包 (END→DISABLE), 要么 HCTO 超时 (COMPARE→DISABLE) */
	{
		uint32_t safety = 0;

		while (NRF_RADIO->EVENTS_DISABLED == 0) {
			if (++safety > 5000000) {
				printk("!STUCK RX! ST=%u EVCC2=%u T0=%u\n",
				       NRF_RADIO->STATE,
				       NRF_RTC0->EVENTS_COMPARE[2],
				       NRF_TIMER0->EVENTS_COMPARE[1]);
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

	/* ★ Save TIMER0 captures BEFORE TX phase
	 * TX EVENTS_READY would overwrite CC[TRX_CC_OFFSET] via PPI */
	last_rx_aa_us = radio_tmr_aa_get();
	last_rx_ready_us = radio_tmr_ready_get();

	/*==============================================================
	 * SW TIFS: RX→TX 转换
	 *
	 * RX END 已触发:
	 *   - PPI_CH_TIMER_CLEAR: Timer1 清零到 0, 继续计数
	 *   - SHORTS: END→DISABLE → Radio DISABLED
	 *
	 * 清除 EVENTS_COMPARE (写后读回确保 APB 生效),
	 * 处理数据, 准备 TX PDU,
	 * 然后启用 PPI_CH_TXEN: Timer1 CC[1]=110μs 时触发 TXEN。
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

	/* LL Control 处理 */
	tx_pdu_pending = false;
	if (new_packet && pdu_data_rx.len > 0) {
		if (pdu_data_rx.ll_id == PDU_DATA_LLID_CTRL) {
			tx_pdu_pending = handle_ll_control(&pdu_data_rx);
		}
	}

	/* 准备 TX 包 */
	if (tx_pdu_pending) {
		tx_pdu_buf.nesn = rx_nesn;
		tx_pdu_buf.sn   = tx_sn;
		radio_pkt_tx_set(&tx_pdu_buf);
	} else {
		build_empty_pdu(&tx_pdu_buf);
		radio_pkt_tx_set(&tx_pdu_buf);
	}

	/* 启用 PPI_CH_TXEN: Timer1 CC[1]=100μs 后触发 TXEN */

	/* TX 诊断: 记录 PPI 启用时 Timer1 值 (确认 < CC[1]) */
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CAPTURE3);
	uint32_t t1_at_ppi_en = nrf_timer_cc_get(NRF_TIMER1, NRF_TIMER_CC_CHANNEL3);
	uint32_t cc1_already = NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN];

	nrf_ppi_channels_enable(NRF_PPI, BIT(PPI_CH_TXEN));

	/* SHORTS 保持 READY_START | END_DISABLE (TX 阶段同样适用) */
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;

	/* 等待 TX 完成: PPI TXEN → READY_START → TX → END_DISABLE → DISABLED */
	{
		uint32_t safety = 0;

		while (NRF_RADIO->EVENTS_DISABLED == 0) {
			if (++safety > 2000000) {
				printk("!STUCK TX! ST=%u T1=%u CC1F=%u\n",
				       NRF_RADIO->STATE,
				       t1_at_ppi_en, cc1_already);
				NRF_RADIO->TASKS_DISABLE = 1;
				while (NRF_RADIO->EVENTS_DISABLED == 0) {
				}
				break;
			}
		}
	}

	/* TX 诊断: 确认 TX END 真的发生了 */
	uint32_t tx_end_fired = NRF_RADIO->EVENTS_END;

	NRF_RADIO->EVENTS_DISABLED = 0;

	/* 清理 */
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_TXEN));
	NRF_RADIO->SHORTS = 0;

	/* 前3个事件打印 TX 诊断 */
	if (conn_event_counter < 3) {
		printk("[TX_DBG] #%u T1@en=%u CC1_pre=%u END=%u "
		       "hdr=0x%02X len=%u\n",
		       conn_event_counter, t1_at_ppi_en,
		       cc1_already, tx_end_fired,
		       *(uint8_t *)&tx_pdu_buf, tx_pdu_buf.len);
	}

	if (tx_pdu_pending) {
		tx_sn ^= 1;
	}

	conn_event_counter++;
	return true;
}

/*===========================================================================
 * 连接态主循环 — 锚点机制的完整演示
 *
 * ★ 这是本 Demo 的核心: 展示锚点如何计算、递推、更新
 *
 * 锚点生命周期:
 *
 *   1. 初始化: 从 CONNECT_IND 计算首次锚点
 *   2. 递推:   每个 connInterval 推进一次 (ticks + remainder 累积)
 *   3. RX成功: 更新锚点 (重置 Window Widening)
 *   4. RX失败: 保持旧锚点, Window Widening 增长
 *   5. 超时:   supervision timeout → 断开连接
 *===========================================================================*/
void conn_loop(void)
{
	/* 初始化连接状态 */
	tx_sn = 0;
	rx_nesn = 0;
	conn_event_counter = 0;
	last_unmapped_chan = 0;
	conn_terminated = false;
	tx_pdu_pending = false;
	conn_event_rx_ok = 0;
	conn_event_rx_timeout = 0;
	conn_event_rx_crc_err = 0;

	/* 初始化锚点追踪器 */
	memset(&anchor, 0, sizeof(anchor));

	radio_configure_conn();

	/* ==== 连接参数计算 ==== */
	uint32_t conn_interval_us = (uint32_t)conn_params.interval * CONN_INT_UNIT_US;
	uint32_t win_size_us = (uint32_t)conn_params.win_size * CONN_INT_UNIT_US;

	/* ★ Window Widening 参数计算
	 *
	 * BLE Core Spec Vol 6, Part B, 4.5.7:
	 *   windowWidening = ((masterSCA + slaveSCA) / 1000000) × timeSinceLastAnchor
	 *
	 * 每个 connInterval 的 WW 增量:
	 *   ww_periodic = ceil((master_ppm + slave_ppm) × interval_us / 1000000) */
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

	/* ==== ★ 首次锚点计算 ====
	 *
	 * 这是整个锚点体系的起点!
	 *
	 * BLE Spec 原始公式 (Vol 6, Part B, 4.5.1):
	 *   firstAnchor = CONNECT_IND_end + transmitWindowOffset + transmitWindowDelay
	 *
	 * Zephyr 实现 (ull_periph.c: ull_periph_setup) 添加了工程余量:
	 *   conn_offset_us = winOffset × 1250
	 *                  + WIN_DELAY_LEGACY (1250μs)
	 *                  - EVENT_TICKER_RES_MARGIN   (RTC 分辨率余量)
	 *                  - EVENT_JITTER              (时钟抖动余量)
	 *                  - radio_rx_ready_delay      (RX 启动提前量)
	 *                  - ww_periodic               (首次 WW)
	 *
	 * 减去这些值是为了让 Radio 提前打开 RX, 确保不会错过 Master 的包。
	 * 实际的 RX 窗口 = [anchor - margins, anchor + margins + ww + win_size]
	 */
	uint32_t conn_offset_us = (uint32_t)conn_params.win_offset * CONN_INT_UNIT_US
				+ WIN_DELAY_LEGACY
				- EVENT_TICKER_RES_MARGIN_US
				- EVENT_JITTER_US
				- radio_rx_ready_delay_get(0, 0)
				- ww_periodic_us;

	anchor.initial_anchor_us = conn_offset_us;

	/* ★ 关键: 把 μs 转换为 RTC ticks + 皮秒余量
	 *
	 * 为什么需要两部分?
	 *   RTC0 运行在 32768 Hz, 1 tick = 30517.578125 ns ≈ 30.52 μs
	 *   μs 级别的偏移量无法精确用整数 tick 表示
	 *   余量记录截断误差, 每次累积, 满一个 tick 时进位
	 *   这就是 Zephyr Ticker 的 "亚 tick 精度" 机制 */
	uint32_t offset_ticks = HAL_TICKER_US_TO_TICKS(conn_offset_us);
	uint32_t offset_remainder_ps = HAL_TICKER_REMAINDER(conn_offset_us);

	uint32_t interval_ticks = HAL_TICKER_US_TO_TICKS(conn_interval_us);
	uint32_t interval_remainder_ps = HAL_TICKER_REMAINDER(conn_interval_us);

	uint32_t ps_per_tick = HAL_TICKER_CNTR_CLK_UNIT_FSEC /
			       HAL_TICKER_FSEC_PER_PSEC;

	/* 首次锚点的绝对 RTC tick */
	uint32_t next_event_rtc = (connect_end_rtc + offset_ticks) &
				  HAL_TICKER_CNTR_MASK;
	uint32_t next_event_remainder_ps = offset_remainder_ps;

	anchor.initial_anchor_rtc = next_event_rtc;
	anchor.current_anchor_rtc = next_event_rtc;
	anchor.last_rx_anchor_rtc = connect_end_rtc;

	/* Supervision timeout */
	uint32_t supervision_timeout_us = (uint32_t)conn_params.timeout * 10000;
	uint32_t last_rx_rtc = connect_end_rtc;

	while (!conn_terminated) {
		/* ★ Window Widening 累积
		 *
		 * 每过一个 connInterval, WW 增加 ww_periodic:
		 *   ww = (masterSCA + slaveSCA) × N × connInterval
		 *
		 * 直到成功 RX 时重置为 0 */
		ww_event_us += ww_periodic_us;
		if (ww_event_us > ww_max_us) {
			ww_event_us = ww_max_us;
		}
		anchor.ww_current_us = ww_event_us;

		/* HCTO 窗口 = 2×(jitter + margin + ww) + win_size
		 * 对称地向两侧扩展, 所以乘 2 */
		uint32_t hcto_add = ((EVENT_JITTER_US +
				      EVENT_TICKER_RES_MARGIN_US +
				      ww_event_us) << 1) +
				    win_size_event_us;

		/* 记录当前锚点 */
		anchor.current_anchor_rtc = next_event_rtc;

		/* ==== 执行连接事件 ==== */
		bool rx_ok = conn_event(next_event_rtc,
					next_event_remainder_ps,
					hcto_add);

		int32_t drift_us_log = 0;

		if (rx_ok) {
			/* ★ 锚点漂移校正 (Drift Correction)
			 *
			 * 利用 TIMER0 在 EVENTS_ADDRESS 和 EVENTS_READY 的
			 * 硬件捕获值, 计算 Master 实际发送时刻与我们预期的偏差,
			 * 然后修正锚点, 防止时钟漂移跨事件累积。
			 *
			 * 与 Zephyr ULL ull_drift_ticks_get() 完全相同的算法:
			 *   actual = aa_us - ready_us  (READY→ADDRESS 实际时间)
			 *   expected_center = JIT + MARGIN + ADDR_1M
			 *   drift = actual - expected_center
			 */
			uint32_t actual_s2a = last_rx_aa_us - last_rx_ready_us;
			int32_t drift_us = (int32_t)actual_s2a -
				(int32_t)(EVENT_JITTER_US +
					  EVENT_TICKER_RES_MARGIN_US +
					  ADDR_US_1M);
			drift_us_log = drift_us;

			/* 将 drift 转换为皮秒并修正 remainder */
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

			/* ★ 锚点更新 (Anchor Point Update)
			 *
			 * 漂移校正后, 用校正后的 tick 作为新参考锚点。
			 * 后续事件从这个校正后的锚点推算,
			 * 时钟漂移误差不会跨事件累积! */
			last_rx_rtc = next_event_rtc;
			anchor.last_rx_anchor_rtc = next_event_rtc;
			anchor.anchor_update_count++;
			anchor.consecutive_misses = 0;

			ww_event_us = 0;
			win_size_event_us = 0;
			anchor.ww_current_us = 0;
			anchor.win_size_event_us = 0;
		} else {
			/* ★ 锚点未更新 — Window Widening 继续增长
			 *
			 * 丢包意味着我们不知道 Master 的精确时刻,
			 * 只能从上一个已知锚点推算。
			 * WW 每丢一个包增长一次, RX 窗口越来越宽。 */
			anchor.consecutive_misses++;
			if (anchor.consecutive_misses > anchor.max_consecutive_misses) {
				anchor.max_consecutive_misses = anchor.consecutive_misses;
			}

			/* Supervision timeout 检测 */
			uint32_t elapsed_ticks = (next_event_rtc - last_rx_rtc) &
						 HAL_TICKER_CNTR_MASK;
			uint32_t elapsed_us = HAL_TICKER_TICKS_TO_US(elapsed_ticks);

			if (elapsed_us >= supervision_timeout_us) {
				printk("[ANCHOR] Supervision timeout! no RX for %u us\n",
				       elapsed_us);
				break;
			}
		}

		/* ★ 锚点递推: 推进到下一个 connInterval
		 *
		 * next_anchor = current_anchor + interval_ticks
		 *
		 * 关键细节: 皮秒余量累积与进位
		 *   remainder 每次 += interval_remainder
		 *   当 remainder >= 1 tick 的皮秒数时, 进位 +1 tick
		 *   这防止了舍入误差的长期累积
		 *
		 * 举例 (connInterval = 30ms):
		 *   30000 μs / 30.52 μs = 982.97 ticks
		 *   ticks_interval = 982
		 *   remainder 追踪 0.97 × 30.52μs ≈ 29.6 μs
		 *   每 ~33 个事件 remainder 溢出, 多加 1 tick
		 *   长期平均: 精确等于 30000 μs */
		next_event_rtc = (next_event_rtc + interval_ticks) &
				 HAL_TICKER_CNTR_MASK;
		next_event_remainder_ps += interval_remainder_ps;
		if (next_event_remainder_ps >= ps_per_tick) {
			next_event_remainder_ps -= ps_per_tick;
			next_event_rtc = (next_event_rtc + 1) &
					 HAL_TICKER_CNTR_MASK;
		}

		uint32_t current_event = conn_event_counter;

		/* 锚点状态日志:
		 * 仅打印前 5 个事件和出错事件, 避免 UART 阻塞 */
		if (current_event <= 5 ||
		    anchor.consecutive_misses > 3) {
			printk("[ANCHOR] #%u ch=%u %s | "
			       "rtc=%u ww=%u drift=%d | "
			       "rx=%u/%u miss=%u\n",
			       current_event,
			       last_unmapped_chan,
			       rx_ok ? "OK" : "MISS",
			       anchor.current_anchor_rtc,
			       anchor.ww_current_us,
			       drift_us_log,
			       conn_event_rx_ok,
			       current_event,
			       anchor.consecutive_misses);
		}
	}

	/* 清理 */
	NRF_RADIO->SHORTS = 0;
	NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;

	printk("\n========== Connection Summary ==========\n");
	printk("  Total events:   %u\n", conn_event_counter);
	printk("  RX OK:          %u\n", conn_event_rx_ok);
	printk("  RX timeout:     %u\n", conn_event_rx_timeout);
	printk("  RX CRC error:   %u\n", conn_event_rx_crc_err);
	printk("  Anchor updates: %u\n", anchor.anchor_update_count);
	printk("  Max consecutive misses: %u\n", anchor.max_consecutive_misses);
	printk("=========================================\n");

	/* 连接参数和锚点计算详情 (连接结束后打印, 避免影响活跃连接) */
	printk("[STATE] Connection state details:\n");
	print_connect_ind();
	print_anchor_calculation(conn_offset_us,
				 offset_ticks,
				 offset_remainder_ps,
				 interval_ticks,
				 interval_remainder_ps,
				 master_sca_ppm,
				 combined_sca_ppm,
				 ww_periodic_us,
				 conn_interval_us);
}
