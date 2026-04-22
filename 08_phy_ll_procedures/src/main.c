/*
 * BLE PHY LL Procedures (link-layer procedures + Instant) Demo
 *
 * This demo adds LL Procedures on top of 07_phy_slave_latency:
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  What is the Instant mechanism?                                 │
 * │                                                                 │
 * │  Some BLE link-layer operations require the Master and Slave    │
 * │  to switch parameters in sync. A Master LL Control PDU carries  │
 * │  an instant value meaning:                                      │
 * │    "new parameters take effect when connEventCounter == instant"│
 * │                                                                 │
 * │  This demo implements two procedures that use Instant:          │
 * │    1. LL_CONNECTION_UPDATE_IND - update interval/latency/timeout│
 * │    2. LL_CHANNEL_MAP_IND       - update the channel map         │
 * │                                                                 │
 * │  Key constraints:                                               │
 * │    - Keep using old parameters until Instant arrives            │
 * │    - Disable Slave Latency while a procedure is pending         │
 * │    - Disconnect if Instant expires (protocol error)             │
 * └─────────────────────────────────────────────────────────────────┘
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
	0x08, 0x09, 'L', 'L', 'P', 'r', 'o', 'c'
};
uint8_t adv_data_len = sizeof(adv_data);

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
