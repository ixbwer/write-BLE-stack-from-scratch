/*
 * BLE PHY First Anchor (首次锚点) Demo
 *
 * 本 Demo 在 03_phy_connect_ind 基础上, 实现了:
 *   1. 从 CONNECT_IND 参数计算首次锚点 A0
 *   2. 通过 RTC0→TIMER0→PPI→RXEN 硬件链在精确时刻启动 RX
 *   3. 执行一个连接事件 (RX Master 数据包 + TX 空 PDU)
 *   4. 打印锚点计算 6 步详细过程
 *
 * 锚点公式:
 *
 *   A0 = T0 + winOffset × 1.25ms + transmitWindowDelay(1.25ms) - margins
 *
 *   其中 margins = TICKER_RES_MARGIN + JITTER + rx_ready_delay + ww_periodic
 *   (提前开 RX, 宁早勿迟)
 *
 * 硬件调度链 (零 CPU 延迟):
 *
 *   NRF_RTC0 ──CC[2]match──> PPI ──> NRF_TIMER0 START
 *   NRF_TIMER0 ──CC[0]=remainder──> PPI ──> RADIO RXEN
 *   RADIO RXEN → 40μs ramp-up → RX START
 *
 * 代码结构:
 *   ble_common.h  — 公共类型 (新增 anchor_tracker, 锚点常量)
 *   hal_radio.c/h — HAL 层 (新增 radio_configure_conn, data_chan_set)
 *   ll_pdu.c/h    — PDU (新增 build_empty_pdu, chan_sel_1, print_anchor_calculation)
 *   adv.c/h       — 广播态 (同 03)
 *   conn.c/h      — 连接态 (新增! conn_first_event: 锚点计算 + 单次事件)
 *   main.c        — 主状态机
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ble_common.h"
#include "hal_radio.h"
#include "ll_pdu.h"
#include "adv.h"
#include "conn.h"

/*===========================================================================
 * 全局变量定义
 *===========================================================================*/

const uint8_t adv_addr[BDADDR_SIZE] = {
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

const uint8_t adv_data[] = {
	0x02, 0x01, 0x06,
	0x08, 0x09, '1', 's', 't', 'A', 'n', 'c', 'h'
};

struct conn_param conn_params;

/* 连接态变量 */
uint8_t tx_sn;
uint8_t rx_nesn;
uint16_t conn_event_counter;
uint8_t last_unmapped_chan;

struct pdu_data tx_pdu_buf __aligned(4);

/* PDU 缓冲区 */
struct pdu_adv pdu_adv_ind __aligned(4);
struct pdu_adv pdu_adv_rx __aligned(4);
struct pdu_data pdu_data_rx __aligned(4);

/* CONNECT_IND 接收时刻 */
uint32_t connect_end_rtc;

/* SCA PPM 映射表 */
const uint16_t sca_ppm_table[] = {500, 250, 150, 100, 75, 50, 30, 20};

/* 锚点追踪 */
struct anchor_tracker anchor;

/* 统计 */
volatile uint32_t adv_event_count;
volatile uint32_t conn_event_rx_ok;
volatile uint32_t conn_event_rx_timeout;

/*===========================================================================
 * 主函数
 *===========================================================================*/
int main(void)
{
	printk("\n");
	printk("===================================================\n");
	printk("  BLE PHY First Anchor Demo\n");
	printk("  计算首次锚点 A0 并执行一个连接事件\n");
	printk("===================================================\n");
	printk("Device: %s  Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
	       ADV_NAME,
	       adv_addr[5], adv_addr[4], adv_addr[3],
	       adv_addr[2], adv_addr[1], adv_addr[0]);
	printk("\n");

	hfclk_start();
	rtc0_start();
	sw_switch_timer_configure();
	ppi_configure();

	while (1) {
		printk("[STATE] Entering ADVERTISING state\n");
		adv_event_count = 0;

		radio_configure_adv();
		build_adv_ind_pdu();

		bool connected = false;

		while (!connected) {
			int ret = adv_event();

			if (ret == 1) {
				connected = true;
			} else {
				k_msleep(ADV_INTERVAL_MS);
			}
		}

		/* ★ 收到 CONNECT_IND! 执行首次连接事件 */
		conn_first_event();

		printk("[STATE] First event done, returning to advertising...\n\n");
		radio_ensure_disabled();
		k_msleep(100);
	}

	return 0;
}
