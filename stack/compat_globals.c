#include "ble_common.h"
#include "l2cap.h"
#include "hci.h"

__weak const uint8_t adv_addr[BDADDR_SIZE] = {
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

__weak uint8_t adv_data[ADV_DATA_LEN_MAX] = {
	0x02, 0x01, 0x06,
	0x0B, 0x09, 'Z', 'e', 'p', 'h', 'y', 'r', 'S', 't', 'a', 'c', 'k'
};

__weak uint8_t adv_data_len = 16;

__weak struct pdu_adv pdu __aligned(4);
__weak struct pdu_adv pdu_adv_ind __aligned(4);
__weak struct pdu_adv pdu_scan_rsp __aligned(4);
__weak struct pdu_adv pdu_rx_buf __aligned(4);
__weak struct pdu_adv pdu_adv_rx __aligned(4);
__weak struct pdu_data pdu_data_rx __aligned(4);
__weak struct pdu_data tx_pdu_buf __aligned(4);

__weak struct conn_param conn_params;
__weak uint8_t tx_sn;
__weak uint8_t rx_nesn;
__weak uint16_t conn_event_counter;
__weak uint8_t last_unmapped_chan;
__weak volatile bool conn_terminated;
__weak uint8_t conn_terminate_reason = 0x13;
__weak bool tx_pdu_pending;

__weak uint32_t connect_end_rtc;
__weak const uint16_t sca_ppm_table[] = {500, 250, 150, 100, 75, 50, 30, 20};
__weak struct anchor_tracker anchor;
__weak struct latency_tracker lat;

__weak struct ll_proc_conn_update proc_conn_update;
__weak struct ll_proc_chan_map_update proc_chan_map;
__weak struct ll_proc_slave_terminate proc_slave_term;
__weak struct ll_proc_slave_conn_param_req proc_slave_cpr;
__weak struct l2cap_cpurq l2cap_cpurq;
__weak struct enc_state enc;
__weak uint8_t ccm_scratch_rx[64] __aligned(4);
__weak uint8_t ccm_scratch_tx[64] __aligned(4);
__weak struct smp_state_s smp_state;

__weak volatile uint32_t adv_event_count;
__weak volatile uint32_t scan_req_count;
__weak volatile uint32_t scan_rsp_count;
__weak volatile uint32_t conn_event_rx_ok;
__weak volatile uint32_t conn_event_rx_timeout;
__weak volatile uint32_t conn_event_rx_crc_err;
__weak volatile uint32_t multi_pdu_total_exchanges;
__weak volatile uint32_t multi_pdu_events_extended;
__weak volatile uint32_t multi_pdu_max_in_event;

__weak uint32_t last_rx_aa_us;
__weak uint32_t last_rx_ready_us;

__weak struct l2cap_reassembly l2cap_reasm;
__weak struct tx_queue data_tx_q;
__weak uint32_t l2cap_rx_count;
__weak uint32_t l2cap_tx_count;

__weak volatile bool hci_adv_enabled;
__weak volatile bool hci_connected;
/* ---------------------------------------------------------------------------
 * Weak function stubs: allow demos that don't link hci.c / l2cap.c to still
 * reference these symbols via conn_event.c, slave_latency.c, conn.c.
 * Strong definitions in hci.c / l2cap.c override these at link time.
 * --------------------------------------------------------------------------- */
__weak void h4_init(void) {}
__weak void h4_process(void) {}
__weak void hci_send_le_conn_complete(void) {}
__weak void hci_send_disconnect_complete(uint8_t reason) { (void)reason; }
__weak void hci_send_num_completed_pkts(uint16_t num) { (void)num; }
__weak void hci_send_acl_data(const uint8_t *data, uint16_t len,
			      bool is_first_fragment)
{
	(void)data; (void)len; (void)is_first_fragment;
}
__weak bool hci_acl_tx_pending(void) { return false; }
__weak uint16_t hci_acl_tx_get(uint8_t *buf, uint16_t buf_size)
{
	(void)buf; (void)buf_size; return 0;
}
__weak bool l2cap_tx_dequeue(struct pdu_data *pdu) { (void)pdu; return false; }
__weak int l2cap_tx_enqueue(const uint8_t *buf, uint16_t total_len)
{
	(void)buf; (void)total_len; return -1;
}
__weak bool l2cap_send_conn_param_update_req(void) { return false; }
__weak void l2cap_init(void) {}
__weak void l2cap_process_complete(void) {}
