/*
 * BLE PHY HCI Transport (HCI H4 UART) Demo
 *
 * This demo adds the HCI H4 transport layer on top of 08_phy_ll_procedures:
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  What is HCI (Host Controller Interface)?                       │
 * │                                                                 │
 * │  In the BLE layered architecture, the Host and Controller can   │
 * │  run on different chips. HCI defines how they communicate:      │
 * │    - Command:  Host -> Controller (Reset, adv config, etc.)     │
 * │    - Event:    Controller -> Host (conn complete, adv reports)  │
 * │    - ACL Data: bidirectional application data                   │
 * │                                                                 │
 * │  H4 is the simplest UART framing for HCI:                       │
 * │    one Packet Indicator byte is added before each packet        │
 * │    0x01 = Command, 0x02 = ACL, 0x04 = Event                     │
 * │                                                                 │
 * │  This demo can be driven from Linux with btattach:              │
 * │    sudo btattach -B /dev/ttyUSB0 -S 115200 -P h4                │
 * │    sudo hciconfig hci0 up                                       │
 * │    hcitool lescan                                               │
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
#include "hci.h"

/*===========================================================================
 * Global definitions
 *===========================================================================*/

const uint8_t adv_addr[BDADDR_SIZE] = {
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

/* In HCI mode, adv_data is configured by the Host with
 * LE_Set_Advertising_Data. A default payload is provided so advertising still
 * works when no Host is attached. */
uint8_t adv_data[ADV_DATA_LEN_MAX] = {
	0x02, 0x01, 0x06,
	0x08, 0x09, 'H', 'C', 'I', '-', 'B', 'L', 'E'
};
uint8_t adv_data_len = 12;

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

/* HCI state */
volatile bool hci_adv_enabled;
volatile bool hci_connected;

int main(void)
{
	printk("\n");
	printk("===================================================\n");
	printk("  BLE PHY HCI Transport Demo\n");
	printk("  HCI H4 UART 传输层 + 空中接口\n");
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

	/* Initialize the HCI UART transport layer. */
	h4_init();

	printk("[HCI] Waiting for Host commands on UART...\n");
	printk("[HCI] (Auto-advertise enabled as fallback)\n");

	/*
	 * Main loop:
	 *
	 * Two modes coexist:
	 *   1. HCI-driven mode: the Host sends HCI_LE_Set_Advertise_Enable
	 *      over UART to start advertising and controls the full flow.
	 *   2. Auto-advertising mode: if no Host command arrives, advertising
	 *      starts automatically as a fallback.
	 *
	 * In both modes, once a connection is established:
	 *   - a LE Connection Complete Event is sent to the Host over UART
	 *   - received air packets are forwarded as HCI ACL Data
	 *   - HCI ACL Data from the Host is transmitted over the air
	 */
	bool auto_adv_started = false;
	uint32_t boot_time = k_uptime_get_32();

	while (1) {
		/* Keep processing HCI commands. */
		h4_process();

		/*
		 * If the Host does not send LE_Set_Advertise_Enable within 3 seconds,
		 * start advertising automatically for standalone testing.
		 */
		bool should_adv = hci_adv_enabled;
		if (!should_adv && !auto_adv_started &&
		    (k_uptime_get_32() - boot_time) > 3000) {
			printk("[HCI] No Host detected, auto-advertising...\n");
			auto_adv_started = true;
			should_adv = true;
		}
		if (!should_adv && auto_adv_started) {
			should_adv = true;
		}

		if (!should_adv) {
			k_msleep(10);
			continue;
		}

		printk("[STATE] Entering ADVERTISING state\n");
		adv_event_count = 0;

		radio_configure_adv();
		build_adv_ind_pdu();

		bool connected = false;
		while (!connected) {
			h4_process();

			/* Stop if the Host has disabled advertising. */
			if (!hci_adv_enabled && !auto_adv_started) {
				break;
			}

			int ret = adv_event();
			if (ret == 1) {
				connected = true;
			} else {
				k_msleep(ADV_INTERVAL_MS);
			}
		}

		if (connected) {
			conn_loop();

			printk("[STATE] Connection lost, returning...\n\n");
			radio_ensure_disabled();
			auto_adv_started = false;
			boot_time = k_uptime_get_32();
			k_msleep(100);
		}
	}

	return 0;
}
