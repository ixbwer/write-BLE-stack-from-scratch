/*
 * BLE PHY LL Procedures Demo — 链路层: PDU 构造、LL Control、信道选择
 *
 * 相比 08_phy_slave_latency 的变化:
 *   1. handle_ll_control() 新增对 LL_CONNECTION_UPDATE_IND 的处理:
 *      解析 Instant 和新连接参数, 存入 proc_conn_update
 *   2. handle_ll_control() 新增对 LL_CHANNEL_MAP_IND 的处理:
 *      解析 Instant 和新 channel map, 存入 proc_chan_map
 *   3. 新增 remap_channel() — 通用信道重映射函数
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

/*===========================================================================
 * 通用信道重映射
 *
 * CSA#1 的 remap 部分: 当 unmapped channel 不在当前 channel map 中时,
 * 通过 remapIndex = unmapped % numUsedChannels 找到替代信道。
 * 单独提取为函数, 方便在 channel map 更新后调用。
 *===========================================================================*/
uint8_t remap_channel(uint8_t unmapped, const uint8_t *chan_map,
		      uint8_t chan_count)
{
	if (chan_map[unmapped >> 3] & (1U << (unmapped & 7))) {
		return unmapped;
	}

	uint8_t remap_index = unmapped % chan_count;
	uint8_t count = 0;

	for (uint8_t i = 0; i < 37; i++) {
		if (chan_map[i >> 3] & (1U << (i & 7))) {
			if (count == remap_index) {
				return i;
			}
			count++;
		}
	}

	return 0;
}

void build_adv_ind_pdu(void)
{
	memset(&pdu_adv_ind, 0, sizeof(pdu_adv_ind));

	pdu_adv_ind.type    = PDU_ADV_TYPE_ADV_IND;
	pdu_adv_ind.tx_addr = 1;
	pdu_adv_ind.chan_sel = 0;
	pdu_adv_ind.rx_addr = 0;
	pdu_adv_ind.len     = BDADDR_SIZE + adv_data_len;

	memcpy(pdu_adv_ind.adv_ind.addr, adv_addr, BDADDR_SIZE);
	memcpy(pdu_adv_ind.adv_ind.data, adv_data, adv_data_len);
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

/*===========================================================================
 * LL Control PDU 处理
 *
 * ★ 新增: LL_CONNECTION_UPDATE_IND (opcode 0x00)
 *   Master 发送, 包含新的连接参数 + Instant:
 *     win_size, win_offset, interval, latency, timeout, instant
 *   Slave 不回复, 只是记下, 在 connEventCounter == instant 时生效。
 *
 * ★ 新增: LL_CHANNEL_MAP_IND (opcode 0x01)
 *   Master 发送, 包含新的 channel map + Instant:
 *     chm[5], instant
 *   Slave 不回复, 在 connEventCounter == instant 时切换 channel map。
 *
 * 这两个 PDU 的共同特点:
 *   1. 只有 Master→Slave 方向
 *   2. Slave 不发送回复 PDU (handle_ll_control 返回 false)
 *   3. 使用 Instant 机制: 在未来某个确定的 connEvent 生效
 *   4. Instant 必须在 (connEventCounter, connEventCounter + 32767) 范围内
 *===========================================================================*/
bool handle_ll_control(const struct pdu_data *rx_pdu)
{
	uint8_t opcode = rx_pdu->llctrl.opcode;

	switch (opcode) {
	case PDU_DATA_LLCTRL_TYPE_CONN_UPDATE_IND: {
		const struct pdu_data_llctrl_conn_update_ind *cu =
			&rx_pdu->llctrl.conn_update_ind;

		proc_conn_update.win_size   = cu->win_size;
		proc_conn_update.win_offset = sys_le16_to_cpu(cu->win_offset);
		proc_conn_update.interval   = sys_le16_to_cpu(cu->interval);
		proc_conn_update.latency    = sys_le16_to_cpu(cu->latency);
		proc_conn_update.timeout    = sys_le16_to_cpu(cu->timeout);
		proc_conn_update.instant    = sys_le16_to_cpu(cu->instant);
		proc_conn_update.pending    = true;

		printk("[LL] RX: LL_CONNECTION_UPDATE_IND\n");
		printk("       interval=%u latency=%u timeout=%u instant=%u\n",
		       proc_conn_update.interval,
		       proc_conn_update.latency,
		       proc_conn_update.timeout,
		       proc_conn_update.instant);

		/* Slave 不回复此 PDU */
		return false;
	}

	case PDU_DATA_LLCTRL_TYPE_CHAN_MAP_IND: {
		const struct pdu_data_llctrl_chan_map_ind *cm =
			&rx_pdu->llctrl.chan_map_ind;

		memcpy(proc_chan_map.chan_map, cm->chm, PDU_CHANNEL_MAP_SIZE);
		proc_chan_map.instant = sys_le16_to_cpu(cm->instant);
		proc_chan_map.pending = true;

		uint8_t new_count = count_ones(proc_chan_map.chan_map,
					       PDU_CHANNEL_MAP_SIZE);
		printk("[LL] RX: LL_CHANNEL_MAP_IND\n");
		printk("       new chan_count=%u instant=%u\n",
		       new_count, proc_chan_map.instant);

		/* Slave 不回复此 PDU */
		return false;
	}

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

/*===========================================================================
 * CSA#1 — 信道选择算法 #1
 *
 * unmapped = (last_unmapped + hop) % 37
 * 如果 unmapped 在当前 chan_map 中 → 使用
 * 否则 remap: index = unmapped % chan_count
 *===========================================================================*/
uint8_t chan_sel_1(void)
{
	uint8_t unmapped;

	unmapped = (last_unmapped_chan + conn_params.hop) % 37;
	last_unmapped_chan = unmapped;

	return remap_channel(unmapped, conn_params.chan_map,
			     conn_params.chan_count);
}
