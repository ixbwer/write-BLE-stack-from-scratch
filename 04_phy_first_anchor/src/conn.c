/*
 * BLE PHY First Anchor Demo — 连接态: 首次连接事件
 *
 * 本文件实现了锚点计算和单个连接事件的执行:
 *
 * ★ conn_first_event() 的流程:
 *   1. 从 CONNECT_IND 参数计算首次锚点 A0
 *   2. 用 chan_sel_1() 计算第一个数据信道
 *   3. 用 radio_tmr_start() 配置 RTC0→TIMER0→PPI→RXEN 硬件链
 *   4. 执行一个 RX→TX 连接事件
 *   5. 打印详细的锚点计算过程
 *
 * ★ 硬件定时链 (零 CPU 延迟):
 *
 *   RTC0 CC[2] match ──PPI──> TIMER0 START
 *   TIMER0 CC[0]=remainder ──PPI──> RADIO RXEN
 *   RADIO RXEN → 40μs ramp → RX START
 *
 *   这条链让 Radio 在精确的锚点时刻开始 RX,
 *   无需 CPU 参与, 消除了中断延迟和任务调度抖动。
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "conn.h"
#include "hal_radio.h"
#include "ll_pdu.h"

/*===========================================================================
 * 单个连接事件
 *
 * 与 06_phy_anchor 的 conn_event() 相同, 但不处理 LL Control PDU。
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

	/* 停止 TIMER0 + 清除 RTC0 比较事件 (防止遗留干扰) */
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
		conn_event_rx_timeout++;
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

	/* 准备 TX: 空 PDU */
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

	printk("[CONN] #%u ch=%u RX OK, TX empty PDU sent\n",
	       conn_event_counter, chan);

	conn_event_counter++;
	return true;
}

/*===========================================================================
 * 首次连接事件 — 锚点计算 + 单次 RX/TX
 *
 * ★ 这是本 Demo 的核心: 展示锚点如何从 CONNECT_IND 参数计算出来
 *
 * 公式 (BLE Core Spec Vol 6, Part B, 4.5.1):
 *   A0 = T0 + transmitWindowOffset + transmitWindowDelay
 *
 * 工程实现加了提前量 (margin):
 *   conn_offset_us = winOffset×1250 + 1250 - margins
 *
 * 减去 margins 是为了让 Radio 提前开 RX, 宁可早到也不能迟到
 *===========================================================================*/
void conn_first_event(void)
{
	/* 初始化连接状态 */
	tx_sn = 0;
	rx_nesn = 0;
	conn_event_counter = 0;
	last_unmapped_chan = 0;
	conn_event_rx_ok = 0;
	conn_event_rx_timeout = 0;

	memset(&anchor, 0, sizeof(anchor));

	radio_configure_conn();

	/* ==== 连接参数计算 ==== */
	uint32_t conn_interval_us = (uint32_t)conn_params.interval * CONN_INT_UNIT_US;
	uint32_t win_size_us = (uint32_t)conn_params.win_size * CONN_INT_UNIT_US;

	/* Window Widening 参数 (仅用于计算, 本例程不动态累积) */
	uint16_t master_sca_ppm = sca_ppm_table[conn_params.sca & 0x07];
	uint32_t combined_sca_ppm = (uint32_t)master_sca_ppm + LOCAL_SCA_PPM;
	uint32_t ww_periodic_us = DIV_ROUND_UP(
		combined_sca_ppm * conn_interval_us, 1000000);

	/* ★ 首次锚点偏移计算 */
	uint32_t conn_offset_us = (uint32_t)conn_params.win_offset * CONN_INT_UNIT_US
				+ WIN_DELAY_LEGACY
				- EVENT_TICKER_RES_MARGIN_US
				- EVENT_JITTER_US
				- radio_rx_ready_delay_get(0, 0)
				- ww_periodic_us;

	anchor.initial_anchor_us = conn_offset_us;

	/* ★ 转换为 RTC ticks + 皮秒余量 (亚 tick 精度) */
	uint32_t offset_ticks = HAL_TICKER_US_TO_TICKS(conn_offset_us);
	uint32_t offset_remainder_ps = HAL_TICKER_REMAINDER(conn_offset_us);

	uint32_t interval_ticks = HAL_TICKER_US_TO_TICKS(conn_interval_us);
	uint32_t interval_remainder_ps = HAL_TICKER_REMAINDER(conn_interval_us);

	/* 首次锚点的绝对 RTC tick */
	uint32_t first_anchor_rtc = (connect_end_rtc + offset_ticks) &
				    HAL_TICKER_CNTR_MASK;

	anchor.initial_anchor_rtc = first_anchor_rtc;
	anchor.current_anchor_rtc = first_anchor_rtc;

	/* HCTO 窗口 (首次事件用 win_size) */
	uint32_t hcto_add = ((EVENT_JITTER_US +
			      EVENT_TICKER_RES_MARGIN_US +
			      ww_periodic_us) << 1) +
			    win_size_us;

	/* ==== 执行首次连接事件 ==== */
	bool rx_ok = conn_event(first_anchor_rtc, offset_remainder_ps, hcto_add);

	printk("\n========== First Connection Event ==========\n");
	printk("  Result: %s\n", rx_ok ? "RX OK" : "RX MISS");
	printk("  RX OK count:     %u\n", conn_event_rx_ok);
	printk("  RX timeout count: %u\n", conn_event_rx_timeout);
	printk("=============================================\n");

	/* 清理 */
	NRF_RADIO->SHORTS = 0;
	NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;

	/* 打印详细的锚点计算过程 (连接事件结束后打印, 避免影响定时) */
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
