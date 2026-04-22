/*
 * BLE PHY Raw TX Demo — using Controller HAL APIs
 *
 * This demo uses the internal HAL abstraction from the Zephyr BLE Controller
 * (radio_phy_set, radio_aa_set, radio_crc_configure, and related helpers)
 * together with the standard struct pdu_adv type to build and transmit a
 * valid BLE advertising packet.
 *
 * PDU Type: ADV_NONCONN_IND (non-connectable undirected advertising)
 * The device "ZephyrRaw" can be discovered with BLE scanner tools such as
 * nRF Connect.
 *
 * Code layout:
 *   ble_common.h  - shared types, constants, and global declarations
 *   hal_radio.c/h - HAL layer: hardware init and radio configuration
 *   main.c        - global definitions, PDU construction, and main loop
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ble_common.h"
#include "hal_radio.h"

/*===========================================================================
 * Global definitions
 *===========================================================================*/

/* Random static address: 66:55:44:33:22:11
 * Stored little-endian (LSB first) inside the BLE PDU. */
const uint8_t adv_addr[BDADDR_SIZE] = {
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

/* AD structures:
 *   Flags: Len=2, Type=0x01, Val=0x06 (LE General Discoverable + BR/EDR Not Supported)
 *   Complete Local Name: Len=10, Type=0x09, Val="ZephyrRaw"
 */
static const uint8_t raw_adv_data[] = {
	0x02, 0x01, 0x06,
	0x0A, 0x09, 'Z', 'e', 'p', 'h', 'y', 'r', 'R', 'a', 'w'
};

#define ADV_DATA_LEN sizeof(raw_adv_data)

/* PDU buffer - uses the controller-standard struct pdu_adv */
struct pdu_adv pdu __aligned(4);

/*===========================================================================
 * Build a standard advertising PDU with struct pdu_adv
 *===========================================================================*/
static void build_adv_pdu(void)
{
	memset(&pdu, 0, sizeof(pdu));

	/* PDU header (encoded by the struct pdu_adv bitfields) */
	pdu.type    = PDU_ADV_TYPE_NONCONN_IND; /* 0x02: non-connectable advertising */
	pdu.tx_addr = 1;                        /* TxAdd=1: Random Address */
	pdu.rx_addr = 0;
	pdu.len     = BDADDR_SIZE + ADV_DATA_LEN;

	/* PDU Payload: AdvA (6 bytes) + AdvData */
	memcpy(pdu.adv_ind.addr, adv_addr, BDADDR_SIZE);
	memcpy(pdu.adv_ind.data, raw_adv_data, ADV_DATA_LEN);

	printk("PDU built: type=0x%02X tx_addr=%d len=%d\n",
	       pdu.type, pdu.tx_addr, pdu.len);
}

/*===========================================================================
 * Main entry
 *===========================================================================*/
int main(void)
{
	printk("=== BLE PHY Raw TX Demo (HAL API) ===\n");
	printk("Device Name: ZephyrRaw\n");
	printk("Address: 66:55:44:33:22:11 (Random Static)\n\n");

	/* Note: this demo does not call bt_enable(); it uses the low-level HAL directly. */

	/* Step 0: start HFCLK, which is required before using RADIO. */
	hfclk_start();

	/* Configure RADIO. */
	radio_configure();

	/* Build the advertising PDU. */
	build_adv_pdu();

	printk("\nStarting advertising on channels 37/38/39...\n");

	int count = 0;

	while (1) {
		count++;

		/* Send on the three advertising channels in order, like a real BLE advertising event. */
		send_on_channel(37);
		send_on_channel(38);
		send_on_channel(39);

		if (count % 10 == 0) {
			printk("Sent %d advertising events\n", count);
		}

		/* ~100 ms advertising interval */
		k_msleep(100);
	}

	return 0;
}
