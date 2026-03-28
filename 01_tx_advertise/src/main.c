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
 * 代码结构:
 *   ble_common.h  — 公共类型、常量与全局变量声明
 *   hal_radio.c/h — HAL 层: 硬件初始化与 Radio 配置
 *   main.c        — 全局变量定义、PDU 构造与主循环
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ble_common.h"
#include "hal_radio.h"

/*===========================================================================
 * 全局变量定义
 *===========================================================================*/

/* 随机静态地址: C6:A5:E4:D3:C2:E1
 * 在 BLE PDU 中以小端存储 (LSB first) */
const uint8_t adv_addr[BDADDR_SIZE] = {
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
struct pdu_adv pdu __aligned(4);

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
