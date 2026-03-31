/*
 * BLE PHY Connection Loop (连接事件循环) Demo
 *
 * 本 Demo 在 04_phy_first_anchor 基础上, 实现了:
 *   1. 多 connInterval 事件循环 (while loop)
 *   2. 锚点递推: next_rtc += interval_ticks + remainder 进位
 *   3. 每事件跳频: chan_sel_1() 计算新数据信道
 *   4. SN/NESN 单 bit 流控 (ACK/新包确认)
 *   5. 基本 supervision timeout 检测
 *
 * 与 06_phy_anchor 的区别:
 *   - Window Widening 固定 (不累积)
 *   - 无 TIMER0 漂移测量与校正
 *   - 无 LL Control PDU 处理
 *   → 连接保持时间有限 (通常 5-15 秒, 取决于晶振精度)
 *
 * 代码结构: 同 04, conn.c 的 conn_first_event() → conn_loop()
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
	0x08, 0x09, 'C', 'o', 'n', 'L', 'o', 'o', 'p'
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
volatile uint32_t conn_event_rx_crc_err;

/*===========================================================================
 * 主函数
 *===========================================================================*/
int main(void)
{
	printk("\n");
	printk("===================================================\n");
	printk("  BLE PHY Connection Loop Demo\n");
	printk("  多事件连接循环: 跳频, SN/NESN, 锚点递推\n");
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

		/* ★ 收到 CONNECT_IND! 进入连接态循环 */
		conn_loop();

		printk("[STATE] Connection ended, returning to advertising...\n\n");
		radio_ensure_disabled();
		k_msleep(100);
	}

	return 0;
}
