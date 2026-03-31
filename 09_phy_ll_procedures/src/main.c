/*
 * BLE PHY LL Procedures (链路层过程 + Instant 机制) Demo
 *
 * 本 Demo 在 08_phy_slave_latency 基础上增加了 LL Procedures 支持:
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  Instant 机制是什么?                                            │
 * │                                                                 │
 * │  BLE 中某些链路层操作需要 Master/Slave 同步切换参数。           │
 * │  Master 发送 LL Control PDU 时携带一个 instant 值,             │
 * │  表示 "在 connEventCounter == instant 时, 新参数生效"。         │
 * │                                                                 │
 * │  本 Demo 实现两个使用 Instant 的过程:                           │
 * │    1. LL_CONNECTION_UPDATE_IND — 更新 interval/latency/timeout  │
 * │    2. LL_CHANNEL_MAP_IND       — 更新 channel map              │
 * │                                                                 │
 * │  关键约束:                                                      │
 * │    - 在 Instant 到达之前, 继续使用旧参数通信                   │
 * │    - 有 pending procedure 时, 禁止 Slave Latency              │
 * │    - Instant 过期 → 断开连接 (协议错误)                        │
 * └─────────────────────────────────────────────────────────────────┘
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
	0x08, 0x09, 'L', 'L', 'P', 'r', 'o', 'c'
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

/* ★ LL Procedure 追踪全局变量 */
struct ll_proc_conn_update proc_conn_update;
struct ll_proc_chan_map_update proc_chan_map;

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
	printk("  BLE PHY LL Procedures Demo\n");
	printk("  演示链路层过程 + Instant 机制\n");
	printk("  (Connection Update / Channel Map Update)\n");
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
