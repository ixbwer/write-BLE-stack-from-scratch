/*
 * BLE PHY Anchor Demo — 广播态: ADV_IND 发送与 CONNECT_IND 接收
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "adv.h"
#include "hal_radio.h"
#include "ll_pdu.h"

/*===========================================================================
 * 广播信道处理
 *===========================================================================*/
int adv_on_channel(uint32_t channel)
{
	uint32_t freq;

	switch (channel) {
	case 37: freq = 2;  break;
	case 38: freq = 26; break;
	case 39: freq = 80; break;
	default: return 0;
	}

	radio_freq_chan_set(freq);
	radio_whiten_iv_set(channel);

	NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;

	/*==============================================================
	 * 第 1 阶段: TX ADV_IND (SW TIFS)
	 *
	 * PPI_CH_RXEN 在 TX 期间保持禁用!
	 * TX END 时 PPI_CH_TIMER_CLEAR 清零 Timer1。
	 * TX END 后启用 PPI_CH_RXEN, 110μs 后 RXEN 触发。
	 *==============================================================*/

	radio_pkt_tx_set(&pdu_adv_ind);

	radio_status_reset();
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_ADDRESS = 0;

	/* SW TIFS: 不使用 DISABLED_RXEN SHORT */
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;

	nrf_timer_cc_set(NRF_TIMER1, (nrf_timer_cc_channel_t)CC_IDX_RXEN,
			 T_IFS_US - RX_RAMP_US);
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN) | BIT(PPI_CH_TXEN));

	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_START);

	NRF_RADIO->TASKS_TXEN = 1;

	/* 等待 TX END */
	while (NRF_RADIO->EVENTS_END == 0) {
	}
	NRF_RADIO->EVENTS_END = 0;

	/*==============================================================
	 * TX→RX 转换 (SW TIFS)
	 *
	 * TX END 时 PPI 清零 Timer1, 从 0 开始计数。
	 * 清除陈旧 COMPARE 事件, 切换到 RX 缓冲区, 启用 PPI_CH_RXEN。
	 * 110μs 后 CC[0] 触发 RXEN → 40μs ramp → RX START (T_IFS=150μs)
	 *==============================================================*/

	NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN] = 0;
	(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN];

	radio_pkt_rx_set(&pdu_adv_rx);

	nrf_ppi_channels_enable(NRF_PPI, BIT(PPI_CH_RXEN));

	/* 等待 TX DISABLED */
	while (NRF_RADIO->EVENTS_DISABLED == 0) {
	}
	NRF_RADIO->EVENTS_DISABLED = 0;

	/*==============================================================
	 * 第 2 阶段: RX 等待 CONNECT_IND
	 *
	 * ★ 必须清除 TX 期间的 ADDRESS 事件!
	 * EVENTS_ADDRESS 在 TX 时被硬件置 1 (AA 发送到空中时),
	 * 若不清除, 后续逻辑会误判为已收到包。
	 *==============================================================*/

	NRF_RADIO->EVENTS_ADDRESS = 0;

	uint32_t timeout_loops = ADV_RX_TIMEOUT_US * 16;
	bool rx_done = false;

	while (timeout_loops > 0) {
		if (NRF_RADIO->EVENTS_DISABLED) {
			rx_done = true;
			break;
		}
		timeout_loops--;
	}

	/* ★ 关键时刻: 立即捕获 T0 — CONNECT_IND 结束时的 RTC0 tick
	 *
	 * 必须在 CRC 检查和 validate 之前读取, 减少软件延迟!
	 * 这个值是整个锚点体系的基准, 所有后续连接事件
	 * 的定时都从这个 T0 开始推算。
	 *
	 * 在 Zephyr 控制器中:
	 *   ftr->ticks_anchor = radio_tmr_start_get() */
	uint32_t rx_end_rtc = NRF_RTC0->COUNTER & HAL_TICKER_CNTR_MASK;

	if (!rx_done) {
		radio_ensure_disabled();
		return 0;
	}

	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN));
	NRF_RADIO->SHORTS = 0;

	if (!radio_crc_is_valid()) {
		return 0;
	}

	if (validate_connect_ind(&pdu_adv_rx)) {
		connect_end_rtc = rx_end_rtc;
		parse_connect_ind(&pdu_adv_rx);
		return 1;
	}

	return 0;
}

/*===========================================================================
 * 广播事件
 *===========================================================================*/
int adv_event(void)
{
	static const uint32_t adv_channels[] = {37, 38, 39};

	for (int i = 0; i < 3; i++) {
		int ret = adv_on_channel(adv_channels[i]);

		if (ret == 1) {
			return 1;
		}
	}

	adv_event_count++;
	if (adv_event_count <= 3 || adv_event_count % 50 == 0) {
		printk("[ADV] events: %u (waiting for connection...)\n",
		       adv_event_count);
	}
	return 0;
}
