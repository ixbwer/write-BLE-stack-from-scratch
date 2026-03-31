/*
 * BLE PHY 软件 TIFS Demo — HAL 层: 硬件初始化与 Radio/Timer/PPI 配置
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
 * 配置 RADIO (注意: 不设置 NRF_RADIO->TIFS!)
 *===========================================================================*/
void radio_configure(void)
{
	/* 清除所有 Radio 中断 */
	NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;
	NRF_RADIO->SHORTS = 0;

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

	/* ★ 注意: SW TIFS 模式下不使用 NRF_RADIO->TIFS 寄存器!
	 * TIFS 完全由 TIMER1 + PPI 控制。
	 * 清零 TIFS 寄存器确保不会意外影响时序。 */
	NRF_RADIO->TIFS = 0;

	printk("Radio configured: BLE 1M (SW TIFS mode, TIFS register=0)\n");
}

/*===========================================================================
 * 配置 SW_SWITCH_TIMER (NRF_TIMER1)
 *
 * 在真实 Controller 中:
 *   SW_SWITCH_TIMER = NRF_TIMER1 (非 single-timer 模式)
 *   radio_tmr_start() 中配置:
 *     SW_SWITCH_TIMER->MODE = 0;          (Timer mode)
 *     SW_SWITCH_TIMER->PRESCALER = 4;     (1MHz, 1μs)
 *     SW_SWITCH_TIMER->BITMODE = 0;       (16-bit)
 *===========================================================================*/
void sw_switch_timer_configure(void)
{
	/* 停止 Timer1 以便安全配置 */
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_STOP);
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);

	/* 禁用 Timer1 所有中断 */
	NRF_TIMER1->INTENCLR = 0xFFFFFFFFUL;

	/* 清除所有 CC 比较事件 */
	for (int i = 0; i < 6; i++) {
		NRF_TIMER1->EVENTS_COMPARE[i] = 0;
	}

	/* Timer 模式, 1MHz 精度 (16MHz / 2^4 = 1MHz → 1μs 分辨率) */
	NRF_TIMER1->MODE      = TIMER_MODE_MODE_Timer;
	NRF_TIMER1->PRESCALER = 4;
	NRF_TIMER1->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;

	printk("SW_SWITCH_TIMER (TIMER1) configured: 1MHz, 16-bit\n");
}

/*===========================================================================
 * 配置 PPI 通道
 *
 * 在真实 Controller 中 (nRF52, PPI 架构):
 *
 *   hal_sw_switch_timer_clear_ppi_config():
 *     PPI: RADIO END → TIMER CLEAR
 *
 *   hal_radio_txen_on_sw_switch(cc, ppi):
 *     PPI: TIMER COMPARE[cc] → RADIO TXEN
 *
 *   hal_radio_rxen_on_sw_switch(cc, ppi):
 *     PPI: TIMER COMPARE[cc] → RADIO RXEN
 *
 * 我们的简化版本:
 *   - 不使用 PPI Groups (真实 Controller 用 Groups 管理 toggle)
 *   - 直接通过 CHENSET/CHENCLR 启停 PPI 通道
 *===========================================================================*/
void ppi_configure(void)
{
	/* PPI CH0: RADIO EVENTS_END → TIMER1 TASKS_CLEAR
	 *
	 * 功能: 每当 Radio 包传输/接收结束 (END event),
	 *        PPI 自动将 TIMER1 清零到 0 并开始重新计数。
	 *        这是 SW TIFS 的时间基准: t=0 从包结束时刻开始。
	 *
	 * 对应真实 Controller:
	 *   hal_sw_switch_timer_clear_ppi_config()
	 */
	nrf_ppi_channel_endpoint_setup(
		NRF_PPI,
		(nrf_ppi_channel_t)PPI_CH_TIMER_CLEAR,
		(uint32_t)&NRF_RADIO->EVENTS_END,
		(uint32_t)&NRF_TIMER1->TASKS_CLEAR);

	/* PPI CH1: TIMER1 EVENTS_COMPARE[0] → RADIO TASKS_RXEN
	 *
	 * 功能: 当 TIMER1 计数到 CC[0] 时, PPI 自动触发 Radio RXEN。
	 *        用于 TX→RX 转换 (ADV_IND TX 结束后启动 RX 接收 SCAN_REQ)。
	 *
	 * CC[0] = T_IFS - RX_ramp_delay
	 *       = 150 - 40 = 110 μs
	 *
	 * 对应真实 Controller:
	 *   hal_radio_rxen_on_sw_switch(cc, ppi)
	 */
	nrf_ppi_channel_endpoint_setup(
		NRF_PPI,
		(nrf_ppi_channel_t)PPI_CH_RXEN,
		(uint32_t)&NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN],
		(uint32_t)&NRF_RADIO->TASKS_RXEN);

	/* PPI CH2: TIMER1 EVENTS_COMPARE[1] → RADIO TASKS_TXEN
	 *
	 * 功能: 当 TIMER1 计数到 CC[1] 时, PPI 自动触发 Radio TXEN。
	 *        用于 RX→TX 转换 (SCAN_REQ RX 结束后启动 TX 发送 SCAN_RSP)。
	 *
	 * CC[1] = T_IFS - TX_ramp_delay - RX_chain_delay
	 *       = 150 - 40 - 10 = 100 μs
	 *
	 * RX END 事件比空口最后 bit 晚 RX_chain_delay ≈ 10μs,
	 * 必须从 CC 值中扣除, 否则空口 T_IFS 会偏大 10μs。
	 *
	 * 对应真实 Controller:
	 *   hal_radio_txen_on_sw_switch(cc, ppi)
	 */
	nrf_ppi_channel_endpoint_setup(
		NRF_PPI,
		(nrf_ppi_channel_t)PPI_CH_TXEN,
		(uint32_t)&NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN],
		(uint32_t)&NRF_RADIO->TASKS_TXEN);

	/* 仅启用 Timer Clear PPI (常开), 其他两个按需启用 */
	nrf_ppi_channels_enable(NRF_PPI, BIT(PPI_CH_TIMER_CLEAR));
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN) | BIT(PPI_CH_TXEN));

	printk("PPI configured:\n");
	printk("  CH%d: RADIO END → TIMER1 CLEAR\n", PPI_CH_TIMER_CLEAR);
	printk("  CH%d: TIMER1 CC[%d] → RADIO RXEN\n", PPI_CH_RXEN, CC_IDX_RXEN);
	printk("  CH%d: TIMER1 CC[%d] → RADIO TXEN\n", PPI_CH_TXEN, CC_IDX_TXEN);
}

/*===========================================================================
 * 确保 Radio 回到 DISABLED 状态
 *===========================================================================*/
void radio_ensure_disabled(void)
{
	NRF_RADIO->SHORTS = 0;

	/* 禁用自动切换 PPI, 防止 Timer CC 匹配后触发 RXEN/TXEN */
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN) | BIT(PPI_CH_TXEN));

	/* 停止 Timer, 防止 CC 事件在空闲期间持续触发 */
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_STOP);

	if (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled) {
		NRF_RADIO->EVENTS_DISABLED = 0;
		NRF_RADIO->TASKS_DISABLE = 1;
		while (NRF_RADIO->EVENTS_DISABLED == 0) {
		}
	}
}
