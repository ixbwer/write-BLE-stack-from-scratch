/*
 * BLE PHY — HAL 层: 硬件初始化与 Radio 配置
 *
 * 所有 Radio/Timer/RTC 操作均通过 nrfx HAL 或 Controller HAL 完成,
 * 不直接访问外设寄存器。
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "hal_radio.h"

static bool adv_channel_to_freq(uint32_t channel, uint32_t *freq)
{
	switch (channel) {
	case 37:
		*freq = 2;
		return true;
	case 38:
		*freq = 26;
		return true;
	case 39:
		*freq = 80;
		return true;
	default:
		return false;
	}
}

bool radio_adv_channel_set(uint32_t channel)
{
	uint32_t freq;

	if (!adv_channel_to_freq(channel, &freq)) {
		return false;
	}

	radio_freq_chan_set(freq);
	radio_whiten_iv_set(channel);

	return true;
}

void radio_irq_disable_all(void)
{
	nrf_radio_int_disable(NRF_RADIO, 0xFFFFFFFFUL);
}

void radio_shorts_ready_start_end_disable_set(void)
{
	nrf_radio_shorts_set(NRF_RADIO,
				     NRF_RADIO_SHORT_READY_START_MASK |
				     NRF_RADIO_SHORT_END_DISABLE_MASK);
}

void radio_shorts_clear(void)
{
	nrf_radio_shorts_set(NRF_RADIO, 0);
}

void radio_sw_switch_timer_start(void)
{
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_START);
}

void radio_sw_switch_ppi_disable(void)
{
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_SW_0) | BIT(PPI_CH_SW_1));
}

bool radio_wait_disabled(uint32_t max_spins, bool force_disable_on_timeout)
{
	uint32_t spins = 0;

	while (!radio_has_disabled()) {
		if (max_spins != 0U && ++spins > max_spins) {
			if (force_disable_on_timeout) {
				radio_disable();
				while (!radio_has_disabled()) {
				}
				nrf_radio_event_clear(NRF_RADIO,
						      NRF_RADIO_EVENT_DISABLED);
			}

			return false;
		}
	}

	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	return true;
}

bool radio_wait_done(uint32_t max_spins)
{
	uint32_t spins = 0;

	while (!radio_is_done()) {
		if (max_spins != 0U && ++spins > max_spins) {
			return false;
		}
	}

	return true;
}

bool radio_wait_address(uint32_t max_spins)
{
	uint32_t spins = 0;

	while (!radio_is_address()) {
		if (max_spins != 0U && ++spins > max_spins) {
			return false;
		}
	}

	return true;
}

bool radio_sw_switch_to_rx(void *rx_pdu, uint32_t wait_spins)
{
	radio_pkt_rx_set(rx_pdu);
	radio_status_reset();
	ble_sw_switch(true);

	return radio_wait_disabled(wait_spins, false);
}

bool radio_sw_switch_to_tx(void *tx_pdu, uint32_t wait_spins)
{
	radio_pkt_tx_set(tx_pdu);
	radio_status_reset();
	ble_sw_switch(false);

	return radio_wait_disabled(wait_spins, false);
}

void radio_sw_switch_cleanup(void)
{
	radio_sw_switch_ppi_disable();
	radio_shorts_clear();
}

void hfclk_start(void)
{
	nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
	while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED)) {
		k_msleep(1);
	}
	printk("HFCLK (16MHz HFXO) started\n");
}

void rtc0_start(void)
{
	nrf_rtc_task_trigger(NRF_RTC0, NRF_RTC_TASK_STOP);
	nrf_rtc_prescaler_set(NRF_RTC0, 0);
	nrf_rtc_task_trigger(NRF_RTC0, NRF_RTC_TASK_CLEAR);
	nrf_rtc_task_trigger(NRF_RTC0, NRF_RTC_TASK_START);
	printk("RTC0 started (32768 Hz)\n");
}

void radio_ensure_disabled(void)
{
	radio_sw_switch_cleanup();
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_STOP);

	if (nrf_radio_state_get(NRF_RADIO) != NRF_RADIO_STATE_DISABLED) {
		nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
		nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
		while (!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_DISABLED)) {
		}
	}
}

void radio_configure_adv(void)
{
	radio_phy_set(0, 0);
	radio_tx_power_set(0);

	radio_pkt_configure(RADIO_PKT_CONF_LENGTH_8BIT,
			    PDU_AC_LEG_PAYLOAD_SIZE_MAX,
			    RADIO_PKT_CONF_FLAGS(
				    RADIO_PKT_CONF_PDU_TYPE_AC,
				    RADIO_PKT_CONF_PHY_LEGACY,
				    RADIO_PKT_CONF_CTE_DISABLED));

	uint32_t aa = sys_cpu_to_le32(PDU_AC_ACCESS_ADDR);
	radio_aa_set((const uint8_t *)&aa);
	radio_crc_configure(PDU_CRC_POLYNOMIAL, PDU_AC_CRC_IV);

	nrf_radio_ifs_set(NRF_RADIO, 0);
}

void radio_configure(void)
{
	radio_configure_adv();
}

void radio_configure_conn(void)
{
	radio_phy_set(0, 0);
	radio_tx_power_set(0);

	radio_pkt_configure(RADIO_PKT_CONF_LENGTH_8BIT,
			    PDU_DC_PAYLOAD_SIZE_MIN,
			    RADIO_PKT_CONF_FLAGS(
				    RADIO_PKT_CONF_PDU_TYPE_DC,
				    RADIO_PKT_CONF_PHY_LEGACY,
				    RADIO_PKT_CONF_CTE_DISABLED));

	radio_aa_set(conn_params.access_addr);

	uint32_t crc_iv = conn_params.crc_init[0] |
			  ((uint32_t)conn_params.crc_init[1] << 8) |
			  ((uint32_t)conn_params.crc_init[2] << 16);
	radio_crc_configure(PDU_CRC_POLYNOMIAL, crc_iv);

	nrf_radio_ifs_set(NRF_RADIO, 0);
}

void sw_switch_timer_configure(void)
{
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_STOP);
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);

	nrf_timer_int_disable(NRF_TIMER1, 0xFFFFFFFFUL);
	for (int i = 0; i < 6; i++) {
		nrf_timer_event_clear(NRF_TIMER1, nrf_timer_compare_event_get(i));
	}

	nrf_timer_mode_set(NRF_TIMER1, NRF_TIMER_MODE_TIMER);
	nrf_timer_prescaler_set(NRF_TIMER1, 4);
	nrf_timer_bit_width_set(NRF_TIMER1, NRF_TIMER_BIT_WIDTH_16);

	printk("SW_SWITCH_TIMER (TIMER1) configured: 1MHz, 16-bit\n");
}

void ppi_configure(void)
{
	nrf_ppi_channel_endpoint_setup(
		NRF_PPI, (nrf_ppi_channel_t)PPI_CH_TIMER_CLEAR,
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_END),
		nrf_timer_task_address_get(NRF_TIMER1, NRF_TIMER_TASK_CLEAR));

	nrf_ppi_channels_enable(NRF_PPI, BIT(PPI_CH_TIMER_CLEAR));
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_SW_0) | BIT(PPI_CH_SW_1));

	printk("PPI configured\n");
}

/*===========================================================================
 * SW TIFS Toggle 机制
 *
 * 与真实 Controller 的 sw_switch() 相同的策略:
 *   交替使用 CC[0] / CC[1], 配合对应的 PPI 通道。
 *   每次 TIFS 切换只清除一个 COMPARE 事件, 无需 read-back。
 *===========================================================================*/
static uint8_t sw_toggle;

void ble_sw_switch_reset(void)
{
	sw_toggle = 0;
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_SW_0) | BIT(PPI_CH_SW_1));
}

void ble_sw_switch(bool tx_to_rx)
{
	uint8_t cc = sw_toggle;
	sw_toggle ^= 1;

	uint32_t delay;
	nrf_radio_task_t task;

	if (tx_to_rx) {
		delay = T_IFS_US - RX_RAMP_US;
		task = NRF_RADIO_TASK_RXEN;
	} else {
		delay = T_IFS_US - TX_RAMP_US - RX_CHAIN_DELAY_1M_US;
		task = NRF_RADIO_TASK_TXEN;
	}

	nrf_timer_cc_set(NRF_TIMER1, (nrf_timer_cc_channel_t)cc, delay);
	nrf_timer_event_clear(NRF_TIMER1, nrf_timer_compare_event_get(cc));

	nrf_ppi_channel_t ppi_en  = (nrf_ppi_channel_t)
		(cc == 0 ? PPI_CH_SW_0 : PPI_CH_SW_1);
	nrf_ppi_channel_t ppi_dis = (nrf_ppi_channel_t)
		(cc == 0 ? PPI_CH_SW_1 : PPI_CH_SW_0);

	nrf_ppi_channel_endpoint_setup(NRF_PPI, ppi_en,
		nrf_timer_event_address_get(NRF_TIMER1,
			nrf_timer_compare_event_get(cc)),
		nrf_radio_task_address_get(NRF_RADIO, task));

	nrf_ppi_channels_disable(NRF_PPI, BIT(ppi_dis));
	nrf_ppi_channels_enable(NRF_PPI, BIT(ppi_en));
}

void data_chan_set(uint8_t chan)
{
	uint32_t freq;

	if (chan <= 10) {
		freq = 4 + (chan * 2U);
	} else if (chan <= 36) {
		freq = 28 + ((chan - 11) * 2U);
	} else {
		return;
	}

	radio_freq_chan_set(freq);
	radio_whiten_iv_set(chan);
}

void send_on_channel(uint32_t channel)
{
	if (!radio_adv_channel_set(channel)) {
		return;
	}

	radio_pkt_tx_set(&pdu);
	radio_status_reset();
	radio_switch_complete_and_disable();
	radio_tx_enable();

	while (!radio_has_disabled()) {
		k_busy_wait(100);
	}

	radio_disable();
}
