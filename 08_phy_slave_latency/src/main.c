/*
 * BLE PHY Slave Latency (从机延迟) Demo
 *
 * 本 Demo 在 07_phy_data_channel 基础上增加了 Slave Latency 功能:
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  Slave Latency 是什么?                                          │
 * │                                                                 │
 * │  正常连接中, Slave 每个 connInterval 都必须唤醒射频。           │
 * │  当 CONNECT_IND 中 connSlaveLatency > 0 时:                     │
 * │    Slave 允许最多跳过 connSlaveLatency 个连接事件不监听,        │
 * │    大幅降低射频功耗。                                           │
 * │                                                                 │
 * │  约束:                                                          │
 * │    1. 有数据要发 → 必须监听 (不能跳过)                         │
 * │    2. 连续跳过达到 latency 上限 → 必须监听                     │
 * │    3. WW 在跳过期间按比例增长                                   │
 * │    4. 信道选择即使跳过也必须推进                                │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * 时间线示意 (latency=3):
 *
 *   Event:  #0    #1    #2    #3    #4    #5    #6    #7
 *   Action: LISTEN skip  skip  skip  LISTEN skip  skip  skip
 *   WW:     0     +1p   +2p   +3p   0     +1p   +2p   +3p
 *
 *   (p = ww_periodic_us, LISTEN 成功后 WW 重置)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ble_common.h"
#include "hal_radio.h"
#include "ll_pdu.h"
#include "adv.h"
#include "conn.h"
#include "l2cap.h"

/*===========================================================================
 * 全局变量定义
 *===========================================================================*/

const uint8_t adv_addr[BDADDR_SIZE] = {
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

const uint8_t adv_data[] = {
	0x02, 0x01, 0x06,
	0x06, 0x09, 'S', 'L', 'a', 't', 'e'
};

struct conn_param conn_params;

uint8_t tx_sn;
uint8_t rx_nesn;
uint16_t conn_event_counter;
uint8_t last_unmapped_chan;
volatile bool conn_terminated;

struct pdu_data tx_pdu_buf __aligned(4);
bool tx_pdu_pending;

struct pdu_adv pdu_adv_ind __aligned(4);
struct pdu_adv pdu_adv_rx __aligned(4);
struct pdu_data pdu_data_rx __aligned(4);

uint32_t connect_end_rtc;
const uint16_t sca_ppm_table[] = {500, 250, 150, 100, 75, 50, 30, 20};

struct anchor_tracker anchor;
struct latency_tracker lat;

volatile uint32_t adv_event_count;
volatile uint32_t conn_event_rx_ok;
volatile uint32_t conn_event_rx_timeout;
volatile uint32_t conn_event_rx_crc_err;

uint32_t last_rx_aa_us;
uint32_t last_rx_ready_us;

struct l2cap_reassembly l2cap_reasm;
struct tx_queue data_tx_q;
uint32_t l2cap_rx_count;
uint32_t l2cap_tx_count;

int main(void)
{
	printk("\n");
	printk("===================================================\n");
	printk("  BLE PHY Slave Latency Demo\n");
	printk("  演示从机延迟 (Slave Latency) 省电机制\n");
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

		conn_loop();

		printk("[STATE] Connection lost, returning to advertising...\n\n");
		radio_ensure_disabled();
		k_msleep(100);
	}

	return 0;
}
