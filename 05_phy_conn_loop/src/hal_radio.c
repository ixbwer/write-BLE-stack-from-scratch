/*
 * BLE PHY Connection Loop Demo — HAL 层: 硬件初始化与 Radio 配置
 *
 * (与 04_phy_first_anchor 相同)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "hal_radio.h"

/*===========================================================================
 * 启动 HFCLK
 *===========================================================================*/
void hfclk_start(void)
{
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;

	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
	}

	printk("HFCLK started\n");
}

/*===========================================================================
 * 启动 RTC0 — 为 radio_tmr_start 提供 32768 Hz 时基
 *===========================================================================*/
void rtc0_start(void)
{
	NRF_RTC0->TASKS_STOP = 1;
	NRF_RTC0->PRESCALER = 0;     /* 32768 Hz, 无分频 */
	NRF_RTC0->TASKS_CLEAR = 1;
	NRF_RTC0->TASKS_START = 1;

	printk("RTC0 started (32768 Hz)\n");
}

/*===========================================================================
 * 确保 Radio 回到 DISABLED 状态 (含 SW TIFS 清理)
 *===========================================================================*/
void radio_ensure_disabled(void)
{
	NRF_RADIO->SHORTS = 0;

	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN) | BIT(PPI_CH_TXEN));

	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_STOP);

	if (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled) {
		NRF_RADIO->EVENTS_DISABLED = 0;
		NRF_RADIO->TASKS_DISABLE = 1;
		while (NRF_RADIO->EVENTS_DISABLED == 0) {
		}
	}
}

/*===========================================================================
 * 配置 Radio — 广播态
 *===========================================================================*/
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

	NRF_RADIO->TIFS = 0;
}

/*===========================================================================
 * 配置 Radio — 连接态
 *===========================================================================*/
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

	NRF_RADIO->TIFS = 0;
}

/*===========================================================================
 * 配置 SW_SWITCH_TIMER (NRF_TIMER1)
 *===========================================================================*/
void sw_switch_timer_configure(void)
{
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_STOP);
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);

	NRF_TIMER1->INTENCLR = 0xFFFFFFFFUL;

	for (int i = 0; i < 6; i++) {
		NRF_TIMER1->EVENTS_COMPARE[i] = 0;
	}

	NRF_TIMER1->MODE      = TIMER_MODE_MODE_Timer;
	NRF_TIMER1->PRESCALER = 4;
	NRF_TIMER1->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;

	printk("SW_SWITCH_TIMER (TIMER1) configured: 1MHz, 16-bit\n");
}

/*===========================================================================
 * 配置 PPI 通道
 *===========================================================================*/
void ppi_configure(void)
{
	nrf_ppi_channel_endpoint_setup(
		NRF_PPI,
		(nrf_ppi_channel_t)PPI_CH_TIMER_CLEAR,
		(uint32_t)&NRF_RADIO->EVENTS_END,
		(uint32_t)&NRF_TIMER1->TASKS_CLEAR);

	nrf_ppi_channel_endpoint_setup(
		NRF_PPI,
		(nrf_ppi_channel_t)PPI_CH_RXEN,
		(uint32_t)&NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN],
		(uint32_t)&NRF_RADIO->TASKS_RXEN);

	nrf_ppi_channel_endpoint_setup(
		NRF_PPI,
		(nrf_ppi_channel_t)PPI_CH_TXEN,
		(uint32_t)&NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN],
		(uint32_t)&NRF_RADIO->TASKS_TXEN);

	nrf_ppi_channels_enable(NRF_PPI, BIT(PPI_CH_TIMER_CLEAR));
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN) | BIT(PPI_CH_TXEN));

	printk("PPI configured: CH%d END→CLEAR, CH%d CC[%d]→RXEN, CH%d CC[%d]→TXEN\n",
	       PPI_CH_TIMER_CLEAR, PPI_CH_RXEN, CC_IDX_RXEN, PPI_CH_TXEN, CC_IDX_TXEN);
}

/*===========================================================================
 * 数据信道频率设置
 *===========================================================================*/
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
