/*
 * BLE PHY Anchor Point Mechanism Demo
 *
 * This demo focuses on Anchor Point, the core timing concept in a BLE
 * connection.
 *
 * ┌ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┐
 * │ What is Anchor Point?                                            │
 * │                                                                  │
 * │ Anchor Point is the time reference for each connection event in  │
 * │ a BLE connection. Core Spec Vol 6, Part B, Section 4.5.1 says:  │
 * │   "The anchor point of the first connection event shall be       │
 * │    transmitWindowDelay + transmitWindowOffset + n × connInterval │
 * │    after the end of the CONNECT_IND PDU"                         │
 * │                                                                  │
 * │ Key roles of the anchor point:                                   │
 * │   1. First anchor: computed precisely from CONNECT_IND reception  │
 * │   2. Next anchors: advanced every connInterval (absolute time)   │
 * │   3. Anchor update: calibrated with the actual RX timestamp      │
 * │   4. Window widening: widen RX when clock drift accumulates      │
 * └ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┘
 *
 * Anchor timeline:
 *
 *   CONNECT_IND           1st Anchor        2nd Anchor        3rd Anchor
 *   end time (T0)            (A0)              (A1)              (A2)
 *       │                     │                 │                 │
 *   ────┼─────────────────────┼─────────────────┼─────────────────┼────►
 *       │                     │                 │                 │  time
 *       │← winOffset+winDelay→│←── interval ───→│←── interval ───→│
 *
 *   A0 = T0 + winOffset×1.25ms + transmitWindowDelay(1.25ms)
 *   A1 = A0 + connInterval
 *   A2 = A1 + connInterval
 *   ...
 *   An = A0 + n × connInterval
 *
 * Anchor Point Update:
 *
 *   When the Slave successfully receives a packet from the Master:
 *     - use the packet's actual RX timestamp as the new anchor
 *     - reset Window Widening to 0
 *     - prevent clock drift from accumulating over time
 *
 *   When the Slave keeps missing packets:
 *     - continue extrapolating from the last good anchor + N×interval
 *     - grow Window Widening by (SCA_master + SCA_slave) × interval
 *     - widen the RX window to tolerate larger clock offset
 *
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │              Window Widening illustration                    │
 *   │                                                              │
 *   │   Successful RX: ├─ narrow ─┤      (ww=0, precise alignment) │
 *   │   Miss 1 packet: ├── wider ──┤     (ww += periodic)          │
 *   │   Miss 2 packets:├── even wider ─┤ (ww += periodic)          │
 *   │   ...                                                        │
 *   │   Successful RX: ├─ narrow ─┤      (ww reset, anchor update) │
 *   └──────────────────────────────────────────────────────────────┘
 *
 * Corresponding implementation in the Zephyr controller:
 *
 *   ┌──────────────────────────┬───────────────────────────────────────┐
 *   │  Concept                 │  Zephyr source location               │
 *   │──────────────────────────│───────────────────────────────────────│
 *   │  First anchor calculation│  ull_periph.c: ull_periph_setup()     │
 *   │  Anchor RTC tick form    │  ftr->ticks_anchor = tmr_start_get()  │
 *   │  Anchor to radio start   │  radio_tmr_start(0, ticks, remainder) │
 *   │  connInterval progression│  ticker period params                 │
 *   │  Window Widening growth  │  lll->periph.window_widening_event_us │
 *   │  Anchor update after RX  │  isr_done: ww=0, win_size=0           │
 *   │  HCTO RX timeout guard   │  radio_tmr_hcto_configure(hcto)       │
 *   └──────────────────────────┴───────────────────────────────────────┘
 *
 * This demo adds detailed anchor-related logging so you can see:
 *   - how the first anchor is computed from CONNECT_IND
 *   - the anchor tick value for each connection event
 *   - how Window Widening grows on misses and resets after successful RX
 *   - the exact moment of anchor update
 *
 * Code layout:
 *   ble_common.h  - shared types, constants, and global declarations
 *   hal_radio.c/h - HAL layer: hardware init and radio configuration
 *   ll_pdu.c/h    - link layer: PDU build, LL control, channel selection
 *   adv.c/h       - advertising state: ADV_IND TX and CONNECT_IND RX
 *   conn.c/h      - connected state: events and anchor scheduling
 *   main.c        - global definitions and top-level state machine
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ble_common.h"
#include "hal_radio.h"
#include "ll_pdu.h"
#include "adv.h"
#include "conn.h"

/*===========================================================================
 * Global definitions
 *===========================================================================*/

const uint8_t adv_addr[BDADDR_SIZE] = {
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

uint8_t adv_data[] = {
	0x02, 0x01, 0x06,
	0x07, 0x09, 'A', 'n', 'c', 'h', 'o', 'r'
};
uint8_t adv_data_len = sizeof(adv_data);

/*===========================================================================
 * Main entry
 *===========================================================================*/
int main(void)
{
	printk("\n");
	printk("===================================================\n");
	printk("  BLE PHY Anchor Point Demo\n");
	printk("  演示 BLE 连接中的锚点 (Anchor) 机制\n");
	printk("===================================================\n");
	printk("Device: %s  Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
	       ADV_NAME,
	       adv_addr[5], adv_addr[4], adv_addr[3],
	       adv_addr[2], adv_addr[1], adv_addr[0]);
	printk("\n");
	printk("Anchor Point 生命周期:\n");
	printk("  1. CONNECT_IND → 计算首次锚点 (T0 + offset)\n");
	printk("  2. 每个 connInterval 递推 (ticks + remainder)\n");
	printk("  3. RX 成功 → 更新锚点, WW 重置\n");
	printk("  4. RX 失败 → WW 增长, 窗口加宽\n");
	printk("  5. Supervision timeout → 断开\n");
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
