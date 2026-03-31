/*
 * BLE PHY Data Channel Demo — 链路层: PDU 构造、LL Control、信道选择
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ll_pdu.h"

uint8_t count_ones(const uint8_t *data, uint8_t len)
{
	uint8_t count = 0;

	for (uint8_t i = 0; i < len; i++) {
		uint8_t byte = data[i];
		while (byte) {
			count += byte & 1;
			byte >>= 1;
		}
	}
	return count;
}

void build_adv_ind_pdu(void)
{
	memset(&pdu_adv_ind, 0, sizeof(pdu_adv_ind));

	pdu_adv_ind.type    = PDU_ADV_TYPE_ADV_IND;
	pdu_adv_ind.tx_addr = 1;
	pdu_adv_ind.chan_sel = 0;
	pdu_adv_ind.rx_addr = 0;
	pdu_adv_ind.len     = BDADDR_SIZE + ADV_DATA_LEN;

	memcpy(pdu_adv_ind.adv_ind.addr, adv_addr, BDADDR_SIZE);
	memcpy(pdu_adv_ind.adv_ind.data, adv_data, ADV_DATA_LEN);
}

void build_empty_pdu(struct pdu_data *pdu)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = PDU_DATA_LLID_DATA_CONTINUE;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = 0;
	pdu->len   = 0;
}

void build_version_ind(struct pdu_data *pdu)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = 0;
	pdu->len   = 1 + sizeof(struct pdu_data_llctrl_version_ind);

	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_VERSION_IND;
	pdu->llctrl.version_ind.version_number = 0x0C;
	pdu->llctrl.version_ind.company_id = sys_cpu_to_le16(0xFFFF);
	pdu->llctrl.version_ind.sub_version_number = sys_cpu_to_le16(0x0001);
}

void build_feature_rsp(struct pdu_data *pdu)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = 0;
	pdu->len   = 1 + sizeof(struct pdu_data_llctrl_feature_rsp);

	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_FEATURE_RSP;
	memset(pdu->llctrl.feature_rsp.features, 0, 8);
}

void build_unknown_rsp(struct pdu_data *pdu, uint8_t unknown_type)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = 0;
	pdu->len   = 1 + sizeof(struct pdu_data_llctrl_unknown_rsp);

	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_UNKNOWN_RSP;
	pdu->llctrl.unknown_rsp.type = unknown_type;
}

void build_length_rsp(struct pdu_data *pdu)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = 0;
	pdu->len   = 1 + sizeof(struct pdu_data_llctrl_length_rsp);

	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_LENGTH_RSP;
	pdu->llctrl.length_rsp.max_rx_octets = sys_cpu_to_le16(LL_DATA_MTU_DEFAULT);
	pdu->llctrl.length_rsp.max_rx_time = sys_cpu_to_le16(328);
	pdu->llctrl.length_rsp.max_tx_octets = sys_cpu_to_le16(LL_DATA_MTU_DEFAULT);
	pdu->llctrl.length_rsp.max_tx_time = sys_cpu_to_le16(328);
}

bool handle_ll_control(const struct pdu_data *rx_pdu)
{
	uint8_t opcode = rx_pdu->llctrl.opcode;

	switch (opcode) {
	case PDU_DATA_LLCTRL_TYPE_VERSION_IND:
		build_version_ind(&tx_pdu_buf);
		return true;

	case PDU_DATA_LLCTRL_TYPE_FEATURE_REQ:
	case PDU_DATA_LLCTRL_TYPE_PER_INIT_FEAT_XCHG:
		build_feature_rsp(&tx_pdu_buf);
		return true;

	case PDU_DATA_LLCTRL_TYPE_TERMINATE_IND:
		printk("[LL] RX: LL_TERMINATE_IND\n");
		conn_terminated = true;
		return false;

	case PDU_DATA_LLCTRL_TYPE_PING_REQ:
		memset(&tx_pdu_buf, 0, sizeof(tx_pdu_buf));
		tx_pdu_buf.ll_id = PDU_DATA_LLID_CTRL;
		tx_pdu_buf.nesn  = rx_nesn;
		tx_pdu_buf.sn    = tx_sn;
		tx_pdu_buf.len   = 1;
		tx_pdu_buf.llctrl.opcode = PDU_DATA_LLCTRL_TYPE_PING_RSP;
		return true;

	case PDU_DATA_LLCTRL_TYPE_LENGTH_REQ:
		build_length_rsp(&tx_pdu_buf);
		return true;

	default:
		build_unknown_rsp(&tx_pdu_buf, opcode);
		return true;
	}
}

bool validate_connect_ind(const struct pdu_adv *rx_pdu)
{
	if (rx_pdu->type != PDU_ADV_TYPE_CONNECT_IND) {
		return false;
	}
	if (rx_pdu->len != sizeof(struct pdu_adv_connect_ind)) {
		return false;
	}
	if (memcmp(rx_pdu->connect_ind.adv_addr, adv_addr, BDADDR_SIZE) != 0) {
		return false;
	}
	return true;
}

void parse_connect_ind(const struct pdu_adv *rx_pdu)
{
	const struct pdu_adv_connect_ind *ci = &rx_pdu->connect_ind;

	memcpy(conn_params.peer_addr, ci->init_addr, BDADDR_SIZE);
	memcpy(conn_params.access_addr, ci->access_addr, 4);
	memcpy(conn_params.crc_init, ci->crc_init, 3);

	conn_params.win_size   = ci->win_size;
	conn_params.win_offset = sys_le16_to_cpu(ci->win_offset);
	conn_params.interval   = sys_le16_to_cpu(ci->interval);
	conn_params.latency    = sys_le16_to_cpu(ci->latency);
	conn_params.timeout    = sys_le16_to_cpu(ci->timeout);

	memcpy(conn_params.chan_map, ci->chan_map, PDU_CHANNEL_MAP_SIZE);
	conn_params.hop = ci->hop;
	conn_params.sca = ci->sca;
	conn_params.chan_count = count_ones(conn_params.chan_map,
					    PDU_CHANNEL_MAP_SIZE);
}

void print_connect_ind(void)
{
	uint32_t aa_val = sys_get_le32(conn_params.access_addr);
	uint32_t crc_val = conn_params.crc_init[0] |
			   ((uint32_t)conn_params.crc_init[1] << 8) |
			   ((uint32_t)conn_params.crc_init[2] << 16);

	printk("\n========== CONNECT_IND Received ==========\n");
	printk("  Peer:       %02X:%02X:%02X:%02X:%02X:%02X\n",
	       conn_params.peer_addr[5], conn_params.peer_addr[4],
	       conn_params.peer_addr[3], conn_params.peer_addr[2],
	       conn_params.peer_addr[1], conn_params.peer_addr[0]);
	printk("  AA:         0x%08X\n", aa_val);
	printk("  CRC Init:   0x%06X\n", crc_val);
	printk("  Interval:   %d (x1.25ms = %d us)\n",
	       conn_params.interval,
	       (uint32_t)conn_params.interval * CONN_INT_UNIT_US);
	printk("  Latency:    %d\n", conn_params.latency);
	printk("  Timeout:    %d (x10ms)\n", conn_params.timeout);
	printk("  Hop:        %d\n", conn_params.hop);
	printk("  SCA:        %d (%d ppm)\n", conn_params.sca,
	       sca_ppm_table[conn_params.sca & 0x07]);
	printk("  Chan Count: %d / 37\n", conn_params.chan_count);
	printk("==========================================\n");
}

uint8_t chan_sel_1(void)
{
	uint8_t unmapped;

	unmapped = (last_unmapped_chan + conn_params.hop) % 37;
	last_unmapped_chan = unmapped;

	if (conn_params.chan_map[unmapped >> 3] & (1U << (unmapped & 7))) {
		return unmapped;
	}

	uint8_t remap_index = unmapped % conn_params.chan_count;
	uint8_t count = 0;

	for (uint8_t i = 0; i < 37; i++) {
		if (conn_params.chan_map[i >> 3] & (1U << (i & 7))) {
			if (count == remap_index) {
				return i;
			}
			count++;
		}
	}

	return 0;
}
