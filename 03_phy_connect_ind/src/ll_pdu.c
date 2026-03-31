/*
 * BLE PHY CONNECT_IND Demo — 链路层: PDU 构造与 CONNECT_IND 解析
 *
 * 相比 02_tifs_scan 的变化:
 *   - 移除 SCAN_REQ/SCAN_RSP 相关函数
 *   - 新增 validate_connect_ind(): 验证 CONNECT_IND 包类型、长度、AdvA
 *   - 新增 parse_connect_ind(): 提取 AA/CRC/Interval/Hop/SCA 等 12 个字段
 *   - 新增 print_connect_ind(): 格式化输出所有连接参数
 *   - 新增 count_ones(): 计算 Channel Map 中可用信道数
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ll_pdu.h"

/*===========================================================================
 * 辅助函数: 统计比特数
 *===========================================================================*/
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
 * 构造 ADV_IND PDU
 *===========================================================================*/
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

/*===========================================================================
 * 验证 CONNECT_IND
 *
 * 检查三个条件:
 *   1. PDU 类型是否为 CONNECT_IND
 *   2. 长度是否匹配 (34 字节: InitA(6) + AdvA(6) + LLData(22))
 *   3. AdvA 是否指向我们的地址 (确保不是给别的广播者的)
 *===========================================================================*/
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

/*===========================================================================
 * 解析 CONNECT_IND — 提取 LLData 中的 12 个连接参数字段
 *
 * CONNECT_IND 的 LLData 布局 (BLE Core Spec Vol 6, Part B, 2.3.3.1):
 *
 *   Offset  Size  Field
 *   ──────  ────  ─────────────
 *   0       4     AA (Access Address)
 *   4       3     CRCInit
 *   7       1     WinSize
 *   8       2     WinOffset
 *   10      2     Interval
 *   12      2     Latency
 *   14      2     Timeout
 *   16      5     ChM (Channel Map)
 *   21      1     Hop (5 bits) + SCA (3 bits)
 *
 * 我们利用 Zephyr 的 struct pdu_adv_connect_ind 结构体映射这些字段,
 * 避免手工按字节偏移解析。
 *===========================================================================*/
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

/*===========================================================================
 * 打印 CONNECT_IND 参数
 *
 * 格式化输出所有解析结果, 方便验证连接请求是否正确接收。
 * 每个字段同时打印原始值和换算后的物理量。
 *===========================================================================*/
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
	printk("  WinSize:    %d (x1.25ms = %d us)\n",
	       conn_params.win_size,
	       (uint32_t)conn_params.win_size * CONN_INT_UNIT_US);
	printk("  WinOffset:  %d (x1.25ms = %d us)\n",
	       conn_params.win_offset,
	       (uint32_t)conn_params.win_offset * CONN_INT_UNIT_US);
	printk("  Interval:   %d (x1.25ms = %d us)\n",
	       conn_params.interval,
	       (uint32_t)conn_params.interval * CONN_INT_UNIT_US);
	printk("  Latency:    %d\n", conn_params.latency);
	printk("  Timeout:    %d (x10ms = %d ms)\n", conn_params.timeout,
	       (uint32_t)conn_params.timeout * 10);
	printk("  Hop:        %d\n", conn_params.hop);
	printk("  SCA:        %d (%d ppm)\n", conn_params.sca,
	       sca_ppm_table[conn_params.sca & 0x07]);
	printk("  Chan Count: %d / 37\n", conn_params.chan_count);
	printk("  T0 (RTC0):  %u ticks\n", connect_end_rtc);
	printk("==========================================\n");
}
