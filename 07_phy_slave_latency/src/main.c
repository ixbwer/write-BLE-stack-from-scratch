/*
 * BLE PHY Slave Latency Demo
 *
 * This demo adds Slave Latency on top of 06_phy_data_channel:
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  What is Slave Latency?                                         │
 * │                                                                 │
 * │  In a normal connection, the Slave must wake the radio every    │
 * │  connInterval. When connSlaveLatency > 0 in CONNECT_IND:        │
 * │    the Slave may skip up to connSlaveLatency connection events  │
 * │    without listening, greatly reducing radio power.             │
 * │                                                                 │
 * │  Constraints:                                                   │
 * │    1. Pending TX data -> the Slave must listen                  │
 * │    2. Once the latency limit is reached, it must listen         │
 * │    3. WW grows proportionally while events are skipped          │
 * │    4. Channel selection must still advance on skipped events    │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * Timeline illustration (latency=3):
 *
 *   Event:  #0    #1    #2    #3    #4    #5    #6    #7
 *   Action: LISTEN skip  skip  skip  LISTEN skip  skip  skip
 *   WW:     0     +1p   +2p   +3p   0     +1p   +2p   +3p
 *
 *   (p = ww_periodic_us, WW resets after a successful LISTEN)
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
	0x06, 0x09, 'S', 'L', 'a', 't', 'e'
};
uint8_t adv_data_len = sizeof(adv_data);

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
