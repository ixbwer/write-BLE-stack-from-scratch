/*
 * BLE PHY Encryption Demo — 链路层: PDU 构造、LL Control、信道选择
 *
 * 相比 12_phy_l2cap_sig 的变化:
 *   1. LL_ENC_REQ 从拒绝改为正确处理: 推导 SK, 响应 LL_ENC_RSP
 *   2. 新增 LL_START_ENC_RSP 处理: 完成加密握手
 *   3. 新增 build_enc_rsp(), build_start_enc_req(), build_start_enc_rsp()
 *   4. feature_rsp 声明支持 LE Encryption (bit 0)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ll_pdu.h"
#if BLE_FEATURE_SMP
#include "crypto.h"
#include "smp.h"
#endif

static const uint8_t default_scan_rsp_data[] = {
	0x11, 0x09,
	'Z', 'e', 'p', 'h', 'y', 'r', '-',
	'S', 't', 'a', 'c', 'k', '-', 'B', 'L', 'E'
};

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

void build_adv_ind_pdu_with(struct pdu_adv *pdu, const uint8_t *addr,
			    const uint8_t *data, uint8_t data_len)
{
	memset(pdu, 0, sizeof(*pdu));

	pdu->type    = PDU_ADV_TYPE_ADV_IND;
	pdu->tx_addr = 1;
	pdu->chan_sel = 0;
	pdu->rx_addr = 0;
	pdu->len     = BDADDR_SIZE + data_len;

	memcpy(pdu->adv_ind.addr, addr, BDADDR_SIZE);
	memcpy(pdu->adv_ind.data, data, data_len);
}

void build_adv_ind_pdu(void)
{
	build_adv_ind_pdu_with(&pdu_adv_ind, adv_addr, adv_data, adv_data_len);
}

void build_scan_rsp_pdu_with(struct pdu_adv *pdu, const uint8_t *addr,
			     const uint8_t *data, uint8_t data_len)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->type = PDU_ADV_TYPE_SCAN_RSP;
	pdu->tx_addr = 1;
	pdu->rx_addr = 0;
	pdu->len = BDADDR_SIZE + data_len;

	memcpy(pdu->scan_rsp.addr, addr, BDADDR_SIZE);
	memcpy(pdu->scan_rsp.data, data, data_len);
}

void build_scan_rsp_pdu(void)
{
	build_scan_rsp_pdu_with(&pdu_scan_rsp, adv_addr, default_scan_rsp_data,
				sizeof(default_scan_rsp_data));
}

bool validate_scan_req_for(const struct pdu_adv *rx_pdu, const uint8_t *addr)
{
	if (rx_pdu->type != PDU_ADV_TYPE_SCAN_REQ) {
		return false;
	}

	if (rx_pdu->len != sizeof(struct pdu_adv_scan_req)) {
		return false;
	}

	return memcmp(rx_pdu->scan_req.adv_addr, addr, BDADDR_SIZE) == 0;
}

bool validate_scan_req(const struct pdu_adv *rx_pdu)
{
	return validate_scan_req_for(rx_pdu, adv_addr);
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
	/* Bit 0: LE Encryption
	 * Bit 1: Connection Parameters Request Procedure (BLE 4.1) */
	pdu->llctrl.feature_rsp.features[0] = 0x03;
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
	pdu->llctrl.length_rsp.max_rx_octets = sys_cpu_to_le16(LL_PAYLOAD_OCTETS_MAX);
	pdu->llctrl.length_rsp.max_rx_time = sys_cpu_to_le16(328);
	pdu->llctrl.length_rsp.max_tx_octets = sys_cpu_to_le16(LL_PAYLOAD_OCTETS_MAX);
	pdu->llctrl.length_rsp.max_tx_time = sys_cpu_to_le16(328);
}

/*===========================================================================
 * Slave-Initiated LL Control PDUs
 *===========================================================================*/

void build_terminate_ind(struct pdu_data *pdu, uint8_t reason)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = 0;
	pdu->len   = 1 + sizeof(struct pdu_data_llctrl_terminate_ind);

	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_TERMINATE_IND;
	pdu->llctrl.terminate_ind.error_code = reason;

	printk("[LL] TX: LL_TERMINATE_IND (reason=0x%02X)\n", reason);
}

void build_reject_ext_ind(struct pdu_data *pdu, uint8_t reject_opcode,
			  uint8_t error_code)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = 0;
	pdu->len   = 1 + sizeof(struct pdu_data_llctrl_reject_ext_ind);

	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND;
	pdu->llctrl.reject_ext_ind.reject_opcode = reject_opcode;
	pdu->llctrl.reject_ext_ind.error_code = error_code;

	printk("[LL] TX: LL_REJECT_EXT_IND (opcode=0x%02X err=0x%02X)\n",
	       reject_opcode, error_code);
}

void build_conn_param_req(struct pdu_data *pdu)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = 0;
	pdu->len   = 1 + sizeof(struct pdu_data_llctrl_conn_param_req);

	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_CONN_PARAM_REQ;

	struct pdu_data_llctrl_conn_param_req *req =
		&pdu->llctrl.conn_param_req;

	req->interval_min = sys_cpu_to_le16(proc_slave_cpr.interval_min);
	req->interval_max = sys_cpu_to_le16(proc_slave_cpr.interval_max);
	req->latency      = sys_cpu_to_le16(proc_slave_cpr.latency);
	req->timeout      = sys_cpu_to_le16(proc_slave_cpr.timeout);
	/* preferredPeriodicity = 0 (no preference) */
	req->preferred_periodicity = 0;
	/* referenceConnEventCount = current event counter */
	req->reference_conn_event_count = sys_cpu_to_le16(conn_event_counter);
	/* offset0~5 = 0xFFFF (no preference) */
	req->offset0 = sys_cpu_to_le16(0xFFFF);
	req->offset1 = sys_cpu_to_le16(0xFFFF);
	req->offset2 = sys_cpu_to_le16(0xFFFF);
	req->offset3 = sys_cpu_to_le16(0xFFFF);
	req->offset4 = sys_cpu_to_le16(0xFFFF);
	req->offset5 = sys_cpu_to_le16(0xFFFF);

	printk("[LL] TX: LL_CONNECTION_PARAM_REQ\n");
	printk("       interval=[%u,%u] latency=%u timeout=%u\n",
	       proc_slave_cpr.interval_min, proc_slave_cpr.interval_max,
	       proc_slave_cpr.latency, proc_slave_cpr.timeout);
}

/*===========================================================================
 * Encryption LL Control PDUs
 *===========================================================================*/

void build_enc_rsp(struct pdu_data *pdu)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = 0;
	pdu->len   = 1 + sizeof(struct pdu_data_llctrl_enc_rsp);

	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_ENC_RSP;
	memcpy(pdu->llctrl.enc_rsp.skds, enc.skd_s, 8);
	memcpy(pdu->llctrl.enc_rsp.ivs, enc.iv_s, 4);

	printk("[LL] TX: LL_ENC_RSP\n");
	printk("       SKDs: ");
	for (int i = 0; i < 8; i++) {
		printk("%02X", enc.skd_s[i]);
	}
	printk("  IVs: ");
	for (int i = 0; i < 4; i++) {
		printk("%02X", enc.iv_s[i]);
	}
	printk("\n");
}

void build_start_enc_req(struct pdu_data *pdu)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = 0;
	pdu->len   = 1; /* opcode only, no payload */

	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_START_ENC_REQ;

	printk("[LL] TX: LL_START_ENC_REQ (plaintext)\n");
}

void build_start_enc_rsp(struct pdu_data *pdu)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = 0;
	pdu->len   = 1; /* opcode only, no payload */

	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_START_ENC_RSP;

	printk("[LL] TX: LL_START_ENC_RSP (encrypted)\n");
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
		conn_terminate_reason = rx_pdu->llctrl.terminate_ind.error_code;
		conn_terminated = true;
		return false;

	case PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND: {
		uint8_t rej_opcode = rx_pdu->llctrl.reject_ext_ind.reject_opcode;
		uint8_t err_code = rx_pdu->llctrl.reject_ext_ind.error_code;

		printk("[LL] RX: LL_REJECT_EXT_IND (opcode=0x%02X err=0x%02X)\n",
		       rej_opcode, err_code);

		/* Master 拒绝了我们的 CONN_PARAM_REQ */
		if (rej_opcode == PDU_DATA_LLCTRL_TYPE_CONN_PARAM_REQ) {
			proc_slave_cpr.sent = false;
			proc_slave_cpr.pending = false;
			printk("[LL] Connection Param Request rejected\n");
		}
		return false;
	}

	case PDU_DATA_LLCTRL_TYPE_ENC_REQ:
#if BLE_FEATURE_SMP
	{
		const struct pdu_data_llctrl_enc_req *er =
			&rx_pdu->llctrl.enc_req;

		printk("[LL] RX: LL_ENC_REQ\n");
		printk("       Rand: ");
		for (int i = 0; i < 8; i++) {
			printk("%02X", er->rand[i]);
		}
		printk("  EDIV: %02X%02X\n", er->ediv[1], er->ediv[0]);

		/* 1. 保存 Master 的密钥材料 */
		memcpy(enc.rand, er->rand, 8);
		memcpy(enc.ediv, er->ediv, 2);
		memcpy(enc.skd_m, er->skdm, 8);
		memcpy(enc.iv_m, er->ivm, 4);

		/* 2. 生成 Slave 的随机 SKDs 和 IVs
		 *    (Demo 中使用简易伪随机, 生产级应使用 TRNG) */
		uint32_t seed = NRF_RTC0->COUNTER ^ conn_event_counter;
		for (int i = 0; i < 8; i++) {
			seed = seed * 1103515245 + 12345;
			enc.skd_s[i] = (seed >> 16) & 0xFF;
		}
		for (int i = 0; i < 4; i++) {
			seed = seed * 1103515245 + 12345;
			enc.iv_s[i] = (seed >> 16) & 0xFF;
		}

		/* 3. 构造 16 字节 SKD = SKDm || SKDs
		 *    (SKDm 在低字节, SKDs 在高字节) */
		uint8_t skd[16];
		memcpy(&skd[0], enc.skd_m, 8);
		memcpy(&skd[8], enc.skd_s, 8);

		/* 4. 推导会话密钥: SK = e(LTK, SKD)
		 *    输出大端, CCM 硬件直接使用 */
		crypto_ecb_encrypt(enc.ltk, skd, enc.session_key_be);

		printk("[ENC] SK = e(LTK, SKD) computed\n");

		/* 5. 构造 8 字节 IV = IVm || IVs
		 *    (IVm 在低字节, IVs 在高字节) */
		uint8_t iv[8];
		memcpy(&iv[0], enc.iv_m, 4);
		memcpy(&iv[4], enc.iv_s, 4);

		/* 6. 初始化 CCM 配置:
		 *    - ccm_rx (Master→Slave): direction=1
		 *    - ccm_tx (Slave→Master): direction=0
		 */
		memset(&enc.ccm_rx, 0, sizeof(enc.ccm_rx));
		memset(&enc.ccm_tx, 0, sizeof(enc.ccm_tx));

		memcpy(enc.ccm_rx.key, enc.session_key_be, 16);
		memcpy(enc.ccm_tx.key, enc.session_key_be, 16);

		enc.ccm_rx.counter = 0;
		enc.ccm_tx.counter = 0;

		/* direction 存入 counter 的 bit 39 (Extended 模式) */
		enc.ccm_rx.direction = 1; /* Central → Peripheral */
		enc.ccm_tx.direction = 0; /* Peripheral → Central */

		memcpy(enc.ccm_rx.iv, iv, 8);
		memcpy(enc.ccm_tx.iv, iv, 8);

		/* 7. 响应 LL_ENC_RSP, 进入加密握手 */
		enc.phase = ENC_PHASE_RSP_SENT;
		build_enc_rsp(&tx_pdu_buf);
		return true;
	}
#else
		build_unknown_rsp(&tx_pdu_buf, opcode);
		return true;
#endif

	case PDU_DATA_LLCTRL_TYPE_START_ENC_RSP:
#if BLE_FEATURE_SMP
		printk("[LL] RX: LL_START_ENC_RSP (from Master, encrypted)\n");
		/* Master 的 LL_START_ENC_RSP 到达 = 加密握手完成
		 * 现在开启 TX 加密, 回复我们自己的 LL_START_ENC_RSP (加密) */
		enc.tx_encrypted = true;
		enc.phase = ENC_PHASE_ACTIVE;
		printk("[ENC] ★ Encryption ACTIVE — all PDUs now encrypted\n");		/* \u2605 SMP: \u52a0\u5bc6\u5efa\u7acb\u540e\u5206\u53d1\u5bc6\u94a5 */
		smp_distribute_keys();		build_start_enc_rsp(&tx_pdu_buf);
		return true;
#else
		build_unknown_rsp(&tx_pdu_buf, opcode);
		return true;
#endif

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

bool validate_connect_ind_for(const struct pdu_adv *rx_pdu, const uint8_t *addr)
{
	if (rx_pdu->type != PDU_ADV_TYPE_CONNECT_IND) {
		return false;
	}
	if (rx_pdu->len != sizeof(struct pdu_adv_connect_ind)) {
		return false;
	}
	if (memcmp(rx_pdu->connect_ind.adv_addr, addr, BDADDR_SIZE) != 0) {
		return false;
	}
	return true;
}

bool validate_connect_ind(const struct pdu_adv *rx_pdu)
{
	return validate_connect_ind_for(rx_pdu, adv_addr);
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
