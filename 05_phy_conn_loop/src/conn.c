/*
 * BLE PHY Connection Loop Demo — 连接态: 多事件连接循环
 *
 * 本文件在 04_phy_first_anchor 基础上扩展了:
 *
 * ★ conn_loop() — 多 connInterval 事件循环:
 *   1. 每个 connInterval 推进锚点: next_rtc += interval_ticks
 *   2. 皮秒余量累积与进位: 防止长期舍入漂移
 *   3. 每事件调用 chan_sel_1() 实现跳频
 *   4. 固定 WW (不累积, 每次事件用 ww_periodic_us)
 *   5. 基本 supervision timeout 检测
 *
 * ★ conn_event() 增强 — SN/NESN 流控:
 *   - RX 成功后检查 SN/NESN 以更新流控状态
 *   - 根据 Master 的 NESN 翻转 tx_sn (ACK 机制)
 *   - 根据 Master 的 SN 翻转 rx_nesn (新包确认)
 *
 * 与 06_phy_anchor 的区别:
 *   - 无动态 Window Widening 累积 (WW 固定)
 *   - 无 TIMER0 漂移测量与校正
 *   - 无 LL Control PDU 处理
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "conn.h"
#include "hal_radio.h"
#include "ll_pdu.h"

/*===========================================================================
 * 单个连接事件 (使用 HAL 硬件定时 + SW TIFS)
 *
 * 相比 04 新增: SN/NESN 流控处理
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

	/* 清理上一个事件遗留的 HAL PPI/Timer 状态 */
	radio_tmr_status_reset();

	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;

	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN) | BIT(PPI_CH_TXEN));

	/* 设置 Timer1 CC[1] for RX→TX 转换 */
	nrf_timer_cc_set(NRF_TIMER1, (nrf_timer_cc_channel_t)CC_IDX_TXEN,
			 T_IFS_US - TX_RAMP_US - RX_CHAIN_DELAY_1M_US);

	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_START);

	radio_pkt_rx_set(&pdu_data_rx);

	/* 停止 TIMER0 + 清除 RTC0 比较事件 */
	nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_STOP);
	NRF_RTC0->EVENTS_COMPARE[2] = 0;

	/* ★ 传锚点给硬件: radio_tmr_start 配置 RTC0→TIMER0→PPI→RXEN */
	uint32_t remainder_us = radio_tmr_start(0, ticks_at_start,
						remainder_ps);

	radio_tmr_aa_capture();
	radio_tmr_aa_save(0);

	/* HCTO: 如果在锚点 + HCTO 时间内没收到包头, 自动 DISABLE Radio */
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
				printk("!STUCK RX! ST=%u\n", NRF_RADIO->STATE);
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

	/*==============================================================
	 * SW TIFS: RX→TX 转换
	 *==============================================================*/

	NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN] = 0;
	(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN];
	NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN] = 0;
	(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN];

	/* ★ SN/NESN 流控 (本例程新增)
	 *
	 * BLE Core Spec Vol 6, Part B, 4.5.9:
	 *   - Master 的 NESN 告诉 Slave "我期望你的下一个 SN 是什么"
	 *     如果 master NESN != 我的 tx_sn, 说明 master ACK 了我的包
	 *   - Master 的 SN 告诉 Slave "这是我发的第 N 个包"
	 *     如果 master SN == 我的 rx_nesn, 说明这是新包 */
	if (pdu_data_rx.nesn != tx_sn) {
		tx_sn = pdu_data_rx.nesn;
	}

	if (pdu_data_rx.sn == rx_nesn) {
		rx_nesn ^= 1;
	}

	/* 准备 TX: 空 PDU (with updated SN/NESN) */
	build_empty_pdu(&tx_pdu_buf);
	radio_pkt_tx_set(&tx_pdu_buf);

	/* 启用 PPI_CH_TXEN */
	nrf_ppi_channels_enable(NRF_PPI, BIT(PPI_CH_TXEN));

	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;

	/* 等待 TX 完成 */
	{
		uint32_t safety = 0;

		while (NRF_RADIO->EVENTS_DISABLED == 0) {
			if (++safety > 2000000) {
				printk("!STUCK TX! ST=%u\n", NRF_RADIO->STATE);
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

	conn_event_counter++;
	return true;
}

/*===========================================================================
 * 连接态主循环 — 多事件循环
 *
 * ★ 这是本 Demo 的核心: 展示锚点递推和跳频
 *
 * 相比 04_phy_first_anchor:
 *   - while 循环执行多个连接事件
 *   - 每 connInterval 推进锚点: next_rtc += interval_ticks
 *   - 皮秒余量累积与进位
 *   - chan_sel_1() 每事件计算新信道
 *   - 固定 WW (不做累积, 简化逻辑)
 *   - 基本 supervision timeout
 *
 * 相比 06_phy_anchor (缺少的):
 *   - 无动态 Window Widening
 *   - 无 TIMER0 漂移测量与校正
 *   - 无 LL Control PDU 处理
 *===========================================================================*/
void conn_loop(void)
{
	/* 初始化连接状态 */
	tx_sn = 0;
	rx_nesn = 0;
	conn_event_counter = 0;
	last_unmapped_chan = 0;
	conn_event_rx_ok = 0;
	conn_event_rx_timeout = 0;
	conn_event_rx_crc_err = 0;

	memset(&anchor, 0, sizeof(anchor));

	radio_configure_conn();

	/* ==== 连接参数计算 ==== */
	uint32_t conn_interval_us = (uint32_t)conn_params.interval * CONN_INT_UNIT_US;
	uint32_t win_size_us = (uint32_t)conn_params.win_size * CONN_INT_UNIT_US;

	/* Window Widening 参数 */
	uint16_t master_sca_ppm = sca_ppm_table[conn_params.sca & 0x07];
	uint32_t combined_sca_ppm = (uint32_t)master_sca_ppm + LOCAL_SCA_PPM;
	uint32_t ww_periodic_us = DIV_ROUND_UP(
		combined_sca_ppm * conn_interval_us, 1000000);

	/* ★ 首次锚点偏移计算 (同 04) */
	uint32_t conn_offset_us = (uint32_t)conn_params.win_offset * CONN_INT_UNIT_US
				+ WIN_DELAY_LEGACY
				- EVENT_TICKER_RES_MARGIN_US
				- EVENT_JITTER_US
				- radio_rx_ready_delay_get(0, 0)
				- ww_periodic_us;

	anchor.initial_anchor_us = conn_offset_us;

	/* 转换为 RTC ticks + 皮秒余量 */
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

	/* Supervision timeout */
	uint32_t supervision_timeout_us = (uint32_t)conn_params.timeout * 10000;
	uint32_t last_rx_rtc = connect_end_rtc;

	/* ★ 固定 HCTO 窗口 (不累积 WW)
	 * 与 06 不同: 06 每丢一个包 WW 增长, 这里用固定的 ww_periodic_us */
	uint32_t hcto_add = ((EVENT_JITTER_US +
			      EVENT_TICKER_RES_MARGIN_US +
			      ww_periodic_us) << 1) +
			    win_size_us;

	/* ==== 连接事件循环 ==== */
	while (1) {
		/* 记录当前锚点 */
		anchor.current_anchor_rtc = next_event_rtc;

		/* 执行连接事件 */
		bool rx_ok = conn_event(next_event_rtc,
					next_event_remainder_ps,
					hcto_add);

		if (rx_ok) {
			last_rx_rtc = next_event_rtc;
			anchor.anchor_update_count++;
			anchor.consecutive_misses = 0;
		} else {
			anchor.consecutive_misses++;
			if (anchor.consecutive_misses > anchor.max_consecutive_misses) {
				anchor.max_consecutive_misses = anchor.consecutive_misses;
			}

			/* ★ Supervision timeout 检测
			 *
			 * BLE Core Spec Vol 6, Part B, 4.5.2:
			 *   如果超过 connSupervisionTimeout 没成功收到包 → 断开
			 *
			 * 注意: 这里用固定 WW, 没有漂移校正,
			 * 所以连接保持时间有限 (通常 5-15 秒) */
			uint32_t elapsed_ticks = (next_event_rtc - last_rx_rtc) &
						 HAL_TICKER_CNTR_MASK;
			uint32_t elapsed_us = HAL_TICKER_TICKS_TO_US(elapsed_ticks);

			if (elapsed_us >= supervision_timeout_us) {
				printk("[CONN] Supervision timeout! no RX for %u us\n",
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
		 *   这防止了舍入误差的长期累积 */
		next_event_rtc = (next_event_rtc + interval_ticks) &
				 HAL_TICKER_CNTR_MASK;
		next_event_remainder_ps += interval_remainder_ps;
		if (next_event_remainder_ps >= ps_per_tick) {
			next_event_remainder_ps -= ps_per_tick;
			next_event_rtc = (next_event_rtc + 1) &
					 HAL_TICKER_CNTR_MASK;
		}

		uint32_t current_event = conn_event_counter;

		/* 事件日志: 前 10 个 + 出错事件 + 每 50 个 */
		if (current_event <= 10 ||
		    anchor.consecutive_misses > 3 ||
		    current_event % 50 == 0) {
			printk("[CONN] #%u ch=%u %s | "
			       "rtc=%u | rx=%u/%u miss=%u\n",
			       current_event,
			       last_unmapped_chan,
			       rx_ok ? "OK" : "MISS",
			       anchor.current_anchor_rtc,
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

	/* 连接参数和锚点计算详情 */
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
