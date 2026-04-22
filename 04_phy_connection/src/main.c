/*
 * BLE PHY Connection Demo
 *
 * Multi-event connection loop: anchor progression, channel hopping,
 * SN/NESN flow control, and supervision timeout
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ble_common.h"
#include "hal_radio.h"
#include "ll_pdu.h"
#include "adv.h"
#include "conn.h"
#include "hci.h"

/*===========================================================================
 * Global definitions (demo-specific override)
 *
 * Shared globals (conn_params, pdu buffers, anchor, counters, ...) use the
 * weak default definitions in stack/compat_globals.c — do NOT re-define them
 * here, the struct layouts live in stack/ble_common.h.
 *===========================================================================*/

/* Advertising name = "ConLoop" (overrides the weak default from compat_globals) */
uint8_t adv_data[ADV_DATA_LEN_MAX] = {
	0x02, 0x01, 0x06,
	0x08, 0x09, 'C', 'o', 'n', 'L', 'o', 'o', 'p'
};
uint8_t adv_data_len = 12;

/*===========================================================================
 * Main entry
 *===========================================================================*/
int main(void)
{
	printk("\n");
	printk("===================================================\n");
	printk("  BLE PHY Connection Loop Demo\n");
	printk("  多事件连接循环: 跳频, SN/NESN, 键点递推\n");
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
	h4_init();

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

		/* Received CONNECT_IND - enter the connection state. */
		conn_loop();
		printk("[STATE] Connection ended, returning to advertising...\n\n");
		radio_ensure_disabled();
		k_msleep(100);
	}

	return 0;
}
