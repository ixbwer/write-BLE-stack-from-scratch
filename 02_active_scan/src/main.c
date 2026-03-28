/*
 * BLE PHY 软件 TIFS (SW TIFS) Demo — 全局变量定义与主入口
 *
 * 基于 phy_adv_scan, 本 Demo 的关键改进:
 *
 *   使用 NRF_TIMER1 + PPI 硬件互联替代 NRF_RADIO->TIFS 寄存器,
 *   实现与真实 Controller 中 !CONFIG_BT_CTLR_TIFS_HW 路径相同的
 *   软件 TIFS 机制。
 *
 * 代码结构:
 *   ble_common.h  — 公共类型、常量与全局变量声明
 *   hal_radio.c/h — HAL 层: Radio/Timer/PPI 配置
 *   ll_pdu.c/h    — 链路层 PDU 构造与校验
 *   adv.c/h       — 广播状态机 (SW TIFS ADV_IND + SCAN_RSP)
 *   main.c        — 全局变量定义与主入口
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ble_common.h"
#include "hal_radio.h"
#include "ll_pdu.h"
#include "adv.h"

/*===========================================================================
 * 全局变量定义
 *===========================================================================*/

const uint8_t adv_addr[BDADDR_SIZE] = {
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

/* PDU 缓冲区 */
struct pdu_adv pdu_adv_ind __aligned(4);
struct pdu_adv pdu_scan_rsp __aligned(4);
struct pdu_adv pdu_rx_buf __aligned(4);

/* 统计计数器 */
volatile uint32_t adv_event_count;
volatile uint32_t scan_req_count;
volatile uint32_t scan_rsp_count;

/*===========================================================================
 * 主函数
 *===========================================================================*/
int main(void)
{
	printk("\n=== BLE PHY SW TIFS Demo ===\n");
	printk("使用 TIMER1 + PPI 实现软件 TIFS 定时\n");
	printk("对应真实 Controller 的 !CONFIG_BT_CTLR_TIFS_HW 路径\n\n");

	hfclk_start();
	radio_configure();
	sw_switch_timer_configure();
	ppi_configure();
	build_adv_ind_pdu();
	build_scan_rsp_pdu();

	adv_init();
	adv_start();

	printk("Advertising started (SW TIFS mode: TIMER1 + PPI)\n\n");

	while (1) {
		k_sleep(K_SECONDS(5));
		printk("[Main] alive — adv: %u, scan_req: %u, scan_rsp: %u\n",
		       adv_event_count, scan_req_count, scan_rsp_count);
	}

	return 0;
}
