/*
 * BLE PHY Multi-PDU Connection Event Demo
 *
 * This demo adds Multi-PDU connection event support on top of
 * 09_phy_hci:
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  What is Multi-PDU (the MD bit)?                               │
 * │                                                                 │
 * │  In earlier demos, each connection event performed only one     │
 * │  RX+TX exchange. If either side had more data to send, it had   │
 * │  to wait for the next event. With typical intervals of 7.5 to   │
 * │  100 ms, throughput is very low.                                │
 * │                                                                 │
 * │  The MD (More Data) bit in the data channel header solves this: │
 * │    - MD=1 means "I still have more data"                       │
 * │    - If either side sends MD=1, the peer keeps the event open   │
 * │    - When both sides send MD=0, the event closes                │
 * │    - Multiple PDUs can be exchanged within one event            │
 * │                                                                 │
 * │  This demo shows:                                               │
 * │    1. Detecting the Master and Slave MD bits                    │
 * │    2. Multiple RX-TX exchanges in one event                     │
 * │    3. TX->RX switching via PPI + TIMER1                         │
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
#include "smp.h"
#include "att.h"

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
	0x04, 0x09, 'A', 'T', 'T'
};
uint8_t adv_data_len = 8;

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

struct ll_proc_slave_terminate proc_slave_term;
struct ll_proc_slave_conn_param_req proc_slave_cpr;
struct l2cap_cpurq l2cap_cpurq;

/* ★ SMP state */
struct smp_state_s smp_state;

/* ★ Encryption state */
struct enc_state enc;
uint8_t ccm_scratch_rx[64] __aligned(4);
uint8_t ccm_scratch_tx[64] __aligned(4);

volatile uint32_t adv_event_count;
volatile uint32_t conn_event_rx_ok;
volatile uint32_t conn_event_rx_timeout;
volatile uint32_t conn_event_rx_crc_err;

volatile uint32_t multi_pdu_total_exchanges;
volatile uint32_t multi_pdu_events_extended;
volatile uint32_t multi_pdu_max_in_event;

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
	printk("  BLE PHY GATT Server Demo\n");
	printk("  GATT Server: GAP + GATT + Custom Service (0xFF00)\n");
	printk("  TERMINATE after %ds\n",
	       SLAVE_TERMINATE_DELAY_S);
	printk("===================================================\n");

	/* Initialize ATT. */
	att_init();

	/* Initialize SMP. */
	smp_init();

	/* LTK starts all-zero and is replaced with the STK after pairing. */
	memset(enc.ltk, 0, sizeof(enc.ltk));
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
	 *      over UART to start advertising and control the flow.
	 *   2. Auto-advertising mode: if no Host command arrives, start
	 *      advertising automatically as a fallback.
	 *
	 * In both modes, once a connection is established:
	 *   - send a LE Connection Complete Event to the Host over UART
	 *   - forward received air packets as HCI ACL Data
	 *   - transmit Host HCI ACL Data over the air
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
