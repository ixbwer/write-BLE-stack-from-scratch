/*
 * BLE PHY Raw TX Demo — 使用 Controller HAL 层接口
 *
 * 本 Demo 使用 Zephyr BLE Controller 内部的 HAL 抽象层接口
 * （radio_phy_set, radio_aa_set, radio_crc_configure 等）
 * 以及标准 struct pdu_adv 构建并发送合规的 BLE 广播包。
 *
 * PDU Type: ADV_NONCONN_IND (不可连接非定向广播)
 * 使用 nRF Connect 等 BLE 扫描工具可看到设备 "ZephyrRaw"
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <soc.h>
#include <hal/nrf_clock.h>

/* ---- Controller HAL 层头文件 ---- */
#include "hal/ccm.h"       /* struct ccm (需在 radio.h 之前) */
#include "hal/radio.h"     /* radio_phy_set, radio_aa_set, ... */

/* ---- Controller PDU 结构体定义 ---- */
#include "pdu_df.h"        /* struct pdu_cte_info */
#include "pdu_vendor.h"    /* Nordic 特定的 PDU 扩展 */
#include "pdu.h"           /* struct pdu_adv, PDU_AC_ACCESS_ADDR, ... */

/*===========================================================================
 * 广播数据定义
 *===========================================================================*/

/* 随机静态地址: C6:A5:E4:D3:C2:E1
 * 在 BLE PDU 中以小端存储 (LSB first) */
static const uint8_t adv_addr[BDADDR_SIZE] = {
	0xE1, 0xC2, 0xD3, 0xE4, 0xA5, 0xC6
};

/* AD 结构体:
 *   Flags: Len=2, Type=0x01, Val=0x06 (LE General Discoverable + BR/EDR Not Supported)
 *   Complete Local Name: Len=10, Type=0x09, Val="ZephyrRaw"
 */
static const uint8_t adv_data[] = {
	0x02, 0x01, 0x06,
	0x0A, 0x09, 'Z', 'e', 'p', 'h', 'y', 'r', 'R', 'a', 'w'
};

#define ADV_DATA_LEN sizeof(adv_data)

/* PDU 缓冲区 — 使用 Controller 标准 struct pdu_adv */
static struct pdu_adv pdu __aligned(4);

/*===========================================================================
 * 使用 struct pdu_adv 构造标准广播 PDU
 *===========================================================================*/
static void build_adv_pdu(void)
{
	memset(&pdu, 0, sizeof(pdu));

	/* PDU Header (由 struct pdu_adv 的 bitfield 字段构成) */
	pdu.type    = PDU_ADV_TYPE_NONCONN_IND; /* 0x02: 不可连接广播 */
	pdu.tx_addr = 1;                        /* TxAdd=1: Random Address */
	pdu.rx_addr = 0;
	pdu.len     = BDADDR_SIZE + ADV_DATA_LEN;

	/* PDU Payload: AdvA (6 bytes) + AdvData */
	memcpy(pdu.adv_ind.addr, adv_addr, BDADDR_SIZE);
	memcpy(pdu.adv_ind.data, adv_data, ADV_DATA_LEN);

	printk("PDU built: type=0x%02X tx_addr=%d len=%d\n",
	       pdu.type, pdu.tx_addr, pdu.len);
}

/*===========================================================================
 * 启动 HFCLK (16MHz 外部晶振)
 *
 * nRF52 的 RADIO 外设要求 HFXO (外部 16MHz 晶振) 必须处于运行状态,
 * 否则 PLL 无法锁相, 射频发射的频率/调制不正确, 导致扫描不到。
 * 正常流程中 bt_enable() 会启动 HFCLK, 但我们绕过了协议栈,
 * 所以必须手动启动。
 *===========================================================================*/
static void hfclk_start(void)
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
static void radio_configure(void)
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
static void send_on_channel(uint32_t channel)
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

/*===========================================================================
 * 主函数
 *===========================================================================*/
int main(void)
{
	printk("=== BLE PHY Raw TX Demo (HAL API) ===\n");
	printk("Device Name: ZephyrRaw\n");
	printk("Address: C6:A5:E4:D3:C2:E1 (Random Static)\n\n");

	/* 注意: 我们不调用 bt_enable(), 直接使用底层 HAL */

	/* 第零步: 启动 HFCLK — RADIO 的前置条件 */
	hfclk_start();

	/* 配置 RADIO */
	radio_configure();

	/* 构造广播 PDU */
	build_adv_pdu();

	printk("\nStarting advertising on channels 37/38/39...\n");

	int count = 0;

	while (1) {
		count++;

		/* 在三个广播信道上依次发送 (模拟真实 BLE 广播事件) */
		send_on_channel(37);
		send_on_channel(38);
		send_on_channel(39);

		if (count % 10 == 0) {
			printk("Sent %d advertising events\n", count);
		}

		/* ~100ms 广播间隔 */
		k_msleep(100);
	}

	return 0;
}
