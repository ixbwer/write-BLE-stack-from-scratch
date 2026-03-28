/*
 * BLE PHY Raw TX Demo — HAL 层: 硬件初始化与 Radio 配置
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "hal_radio.h"

/*===========================================================================
 * 启动 HFCLK (16MHz 外部晶振)
 *
 * nRF52 的 RADIO 外设要求 HFXO (外部 16MHz 晶振) 必须处于运行状态,
 * 否则 PLL 无法锁相, 射频发射的频率/调制不正确, 导致扫描不到。
 * 正常流程中 bt_enable() 会启动 HFCLK, 但我们绕过了协议栈,
 * 所以必须手动启动。
 *===========================================================================*/
void hfclk_start(void)
{
	/* 请求启动 HFXO */
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;

	/* 等待 HFXO 稳定 */
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
		/* busy wait */
	}

	printk("HFCLK (16MHz HFXO) started\n");
}

/*===========================================================================
 * 使用 HAL 接口配置 RADIO
 *===========================================================================*/
void radio_configure(void)
{
	/* 1. 设置 PHY 模式: Legacy BLE 1Mbit GFSK */
	radio_phy_set(0, 0);

	/* 2. 设置发射功率: 0 dBm */
	radio_tx_power_set(0);

	/* 3. 配置包格式 (PCNF0 & PCNF1)
	 *    - LENGTH 字段 8 bit
	 *    - 最大载荷 37 bytes (Legacy ADV)
	 *    - PDU 类型 = AC (Advertising Channel)
	 *    - PHY = Legacy
	 *    - CTE = Disabled
	 */
	radio_pkt_configure(RADIO_PKT_CONF_LENGTH_8BIT,
			    PDU_AC_LEG_PAYLOAD_SIZE_MAX,
			    RADIO_PKT_CONF_FLAGS(
				    RADIO_PKT_CONF_PDU_TYPE_AC,
				    RADIO_PKT_CONF_PHY_LEGACY,
				    RADIO_PKT_CONF_CTE_DISABLED));

	/* 4. 设置接入地址 (Access Address)
	 *    广播固定为 0x8E89BED6, 需要以 LE 字节序传入 */
	uint32_t aa = sys_cpu_to_le32(PDU_AC_ACCESS_ADDR);

	radio_aa_set((const uint8_t *)&aa);

	/* 5. 配置 CRC
	 *    - 多项式: x^24 + x^10 + x^6 + x^4 + x^3 + x + 1
	 *    - 广播初始值: 0x555555
	 */
	radio_crc_configure(PDU_CRC_POLYNOMIAL, PDU_AC_CRC_IV);

	printk("Radio configured via HAL: BLE 1M, AA=0x%08X, CRC_IV=0x%06X\n",
	       PDU_AC_ACCESS_ADDR, PDU_AC_CRC_IV);
}

/*===========================================================================
 * 在指定广播信道上发送一包
 *
 * 使用 HAL 接口设置频率、白化、数据指针，
 * 并通过 SHORTS 自动完成 READY→START→END→DISABLE 状态机。
 *===========================================================================*/
void send_on_channel(uint32_t channel)
{
	uint32_t freq;

	/* BLE 信道号 → RF 频率偏移 (相对于 2400 MHz) */
	switch (channel) {
	case 37: freq = 2;  break;  /* 2402 MHz */
	case 38: freq = 26; break;  /* 2426 MHz */
	case 39: freq = 80; break;  /* 2480 MHz */
	default: return;
	}

	/* 设置频率 */
	radio_freq_chan_set(freq);

	/* 设置白化 IV (= BLE 信道号) */
	radio_whiten_iv_set(channel);

	/* 指向 PDU 缓冲区 */
	radio_pkt_tx_set(&pdu);

	/* 清除所有事件标志 */
	radio_status_reset();

	/* 确保 RADIO 中断被禁用 (我们用轮询, 不需要 ISR) */
	NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;

	/* 设置硬件 SHORTS:
	 *   READY → START : PLL 锁相完成后自动开始发送
	 *   END → DISABLE : 数据发送完毕后自动关闭射频
	 */
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;

	/* 触发 TX Ramp-Up (HAL 封装) */
	radio_tx_enable();

	/* 等待射频自动 DISABLE (整个流程由 SHORTS 驱动) */
	while (!radio_has_disabled()) {
		/* busy wait */
	}

	/* 清除 SHORTS */
	NRF_RADIO->SHORTS = 0;
}
