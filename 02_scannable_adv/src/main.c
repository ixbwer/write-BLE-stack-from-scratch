/*
 * BLE PHY Software TIFS (SW TIFS) Demo — global definitions and entry point
 *
 * Based on phy_adv_scan, the key improvement in this demo is:
 *
 *   using NRF_TIMER1 + PPI hardware interconnect instead of the
 *   NRF_RADIO->TIFS register, so the code follows the same software-TIFS
 *   path as a real controller with !CONFIG_BT_CTLR_TIFS_HW.
 *
 * Code layout:
 *   ble_common.h  - shared types, constants, and global declarations
 *   hal_radio.c/h - HAL layer: Radio/Timer/PPI configuration
 *   ll_pdu.c/h    - link-layer PDU construction and validation
 *   adv.c/h       - advertising state machine (SW TIFS ADV_IND + SCAN_RSP)
 *   main.c        - global definitions and entry point
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ble_common.h"
#include "hal_radio.h"
#include "ll_pdu.h"
#include "adv.h"

/*===========================================================================
 * Global definitions
 *===========================================================================*/

const uint8_t adv_addr[BDADDR_SIZE] = {
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

/* PDU buffers */
struct pdu_adv pdu_adv_ind __aligned(4);
struct pdu_adv pdu_scan_rsp __aligned(4);
struct pdu_adv pdu_rx_buf __aligned(4);

/* Statistics counters */
volatile uint32_t adv_event_count;
volatile uint32_t scan_req_count;
volatile uint32_t scan_rsp_count;

/*===========================================================================
 * Main entry
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
