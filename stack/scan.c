/*
 * BLE PHY Passive Scan Demo — 广播信道被动扫描模块
 *
 * 仅监听 37/38/39 三个广播信道, 接收并打印首个完整广告信道 PDU,
 * 不发送 SCAN_REQ, 因此属于 passive scan。
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "scan.h"
#include "hal_radio.h"

#define PASSIVE_SCAN_WINDOW_US 10000
#define PASSIVE_SCAN_POLL_US   10

static struct passive_scan_stats stats;

static const char *pdu_type_name(uint8_t type)
{
	switch (type) {
	case PDU_ADV_TYPE_ADV_IND:
		return "ADV_IND";
	case PDU_ADV_TYPE_DIRECT_IND:
		return "DIRECT_IND";
	case PDU_ADV_TYPE_NONCONN_IND:
		return "NONCONN_IND";
	case PDU_ADV_TYPE_SCAN_REQ:
		return "SCAN_REQ";
	case PDU_ADV_TYPE_SCAN_RSP:
		return "SCAN_RSP";
	case PDU_ADV_TYPE_CONNECT_IND:
		return "CONNECT_IND";
	case PDU_ADV_TYPE_SCAN_IND:
		return "SCAN_IND";
	case PDU_ADV_TYPE_EXT_IND:
		return "EXT_IND";
	default:
		return "UNKNOWN";
	}
}

static const uint8_t *primary_addr(const struct pdu_adv *pdu)
{
	switch (pdu->type) {
	case PDU_ADV_TYPE_ADV_IND:
	case PDU_ADV_TYPE_NONCONN_IND:
	case PDU_ADV_TYPE_SCAN_RSP:
	case PDU_ADV_TYPE_SCAN_IND:
		return pdu->adv_ind.addr;
	case PDU_ADV_TYPE_DIRECT_IND:
		return pdu->direct_ind.adv_addr;
	case PDU_ADV_TYPE_SCAN_REQ:
		return pdu->scan_req.scan_addr;
	case PDU_ADV_TYPE_CONNECT_IND:
		return pdu->connect_ind.init_addr;
	default:
		return NULL;
	}
}

static const uint8_t *secondary_addr(const struct pdu_adv *pdu)
{
	switch (pdu->type) {
	case PDU_ADV_TYPE_DIRECT_IND:
		return pdu->direct_ind.tgt_addr;
	case PDU_ADV_TYPE_SCAN_REQ:
		return pdu->scan_req.adv_addr;
	case PDU_ADV_TYPE_CONNECT_IND:
		return pdu->connect_ind.adv_addr;
	default:
		return NULL;
	}
}

static void account_channel_hit(uint8_t channel)
{
	switch (channel) {
	case 37:
		stats.ch37_hits++;
		break;
	case 38:
		stats.ch38_hits++;
		break;
	case 39:
		stats.ch39_hits++;
		break;
	default:
		break;
	}
}

static void account_pdu_type(uint8_t type)
{
	switch (type) {
	case PDU_ADV_TYPE_ADV_IND:
		stats.adv_ind++;
		break;
	case PDU_ADV_TYPE_DIRECT_IND:
		stats.direct_ind++;
		break;
	case PDU_ADV_TYPE_NONCONN_IND:
		stats.nonconn_ind++;
		break;
	case PDU_ADV_TYPE_SCAN_REQ:
		stats.scan_req++;
		break;
	case PDU_ADV_TYPE_SCAN_RSP:
		stats.scan_rsp++;
		break;
	case PDU_ADV_TYPE_CONNECT_IND:
		stats.connect_ind++;
		break;
	case PDU_ADV_TYPE_SCAN_IND:
		stats.scan_ind++;
		break;
	case PDU_ADV_TYPE_EXT_IND:
		stats.ext_ind++;
		break;
	default:
		stats.unknown++;
		break;
	}
}

static void log_packet(uint8_t channel, const struct pdu_adv *pdu)
{
	const uint8_t *addr = primary_addr(pdu);
	const uint8_t *extra = secondary_addr(pdu);

	if (addr != NULL) {
		printk("[SCAN] ch=%u type=%s len=%u tx=%s addr=%02X:%02X:%02X:%02X:%02X:%02X",
		       channel, pdu_type_name(pdu->type), pdu->len,
		       pdu->tx_addr ? "rnd" : "pub",
		       addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
	} else {
		printk("[SCAN] ch=%u type=%s len=%u tx=%s",
		       channel, pdu_type_name(pdu->type), pdu->len,
		       pdu->tx_addr ? "rnd" : "pub");
	}

	if (extra != NULL) {
		printk(" peer=%02X:%02X:%02X:%02X:%02X:%02X",
		       extra[5], extra[4], extra[3], extra[2], extra[1], extra[0]);
	}

	printk("\n");
}

void passive_scan_init(void)
{
	memset(&stats, 0, sizeof(stats));
	memset(&pdu_rx_buf, 0, sizeof(pdu_rx_buf));
}

bool passive_scan_listen_channel(uint8_t channel)
{
	uint32_t timeout_left = PASSIVE_SCAN_WINDOW_US;

	if (!radio_adv_channel_set(channel)) {
		return false;
	}

	stats.windows++;

	radio_pkt_rx_set(&pdu_rx_buf);
	radio_status_reset();
	radio_irq_disable_all();
	radio_switch_complete_and_disable();
	radio_rx_enable();

	while (!radio_is_done() &&
	       timeout_left > 0) {
		k_busy_wait(PASSIVE_SCAN_POLL_US);
		timeout_left -= PASSIVE_SCAN_POLL_US;
	}

	if (!radio_is_done()) {
		radio_disable();
		while (!radio_has_disabled()) {
		}
		stats.rx_timeouts++;
		return false;
	}

	while (!radio_has_disabled()) {
	}
	radio_shorts_clear();

	if (!radio_crc_is_valid()) {
		stats.crc_errors++;
		return false;
	}

	stats.packets_ok++;
	account_channel_hit(channel);
	account_pdu_type(pdu_rx_buf.type);
	log_packet(channel, &pdu_rx_buf);

	return true;
}

void passive_scan_cycle(void)
{
	(void)passive_scan_listen_channel(37);
	(void)passive_scan_listen_channel(38);
	(void)passive_scan_listen_channel(39);
}

const struct passive_scan_stats *passive_scan_stats_get(void)
{
	return &stats;
}