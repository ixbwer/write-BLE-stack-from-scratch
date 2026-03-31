/*
 * BLE PHY First Anchor Demo — 链路层: PDU 构造、CONNECT_IND 解析、信道选择
 *
 * 相比 03_phy_connect_ind 新增:
 *   - build_empty_pdu(): 构造空数据 PDU (连接事件 TX 响应)
 *   - chan_sel_1(): Channel Selection Algorithm #1
 *   - print_anchor_calculation(): 锚点计算 6 步详细过程输出
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
 * 构造空数据 PDU
 *
 * 连接事件中, 即使没有数据要发, Slave 也必须回一个空 PDU
 * 以维持连接 (告知 Master: 我还在, 且我的 SN/NESN 状态是...)
 *===========================================================================*/
void build_empty_pdu(struct pdu_data *pdu)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = PDU_DATA_LLID_DATA_CONTINUE;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = 0;
	pdu->len   = 0;
}

/*===========================================================================
 * 验证 CONNECT_IND
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
 * 解析 CONNECT_IND
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

/*===========================================================================
 * 打印锚点计算的详细过程 (本 Demo 核心教学输出)
 *===========================================================================*/
void print_anchor_calculation(uint32_t conn_offset_us,
			      uint32_t offset_ticks,
			      uint32_t offset_remainder_ps,
			      uint32_t interval_ticks,
			      uint32_t interval_remainder_ps,
			      uint16_t master_sca_ppm,
			      uint32_t combined_sca_ppm,
			      uint32_t ww_periodic_us,
			      uint32_t conn_interval_us)
{
	printk("\n===== Anchor Point Calculation =====\n");
	printk("\n[Step 1] T0: CONNECT_IND end time\n");
	printk("  connect_end_rtc = %u (RTC0 ticks, 32768 Hz)\n",
	       connect_end_rtc);
	printk("  T0 = %u x 30.52us = ~%u us\n",
	       connect_end_rtc,
	       HAL_TICKER_TICKS_TO_US(connect_end_rtc));

	printk("\n[Step 2] Calculate first anchor offset\n");
	printk("  BLE Spec formula:\n");
	printk("    A0 = T0 + winOffset*1.25ms + transmitWindowDelay\n");
	printk("  Implementation adds margins:\n");
	printk("    conn_offset_us = winOffset*1250 + WIN_DELAY(1250)\n");
	printk("                   - TICKER_RES_MARGIN(%d)\n",
	       EVENT_TICKER_RES_MARGIN_US);
	printk("                   - EVENT_JITTER(%d)\n",
	       EVENT_JITTER_US);
	printk("                   - rx_ready_delay(%d)\n",
	       radio_rx_ready_delay_get(0, 0));
	printk("                   - ww_periodic(%d)\n", ww_periodic_us);
	printk("    = %d*1250 + 1250 - %d - %d - %d - %d\n",
	       conn_params.win_offset,
	       EVENT_TICKER_RES_MARGIN_US,
	       EVENT_JITTER_US,
	       radio_rx_ready_delay_get(0, 0),
	       ww_periodic_us);
	printk("    = %u us\n", conn_offset_us);

	printk("\n[Step 3] Convert to RTC ticks + remainder\n");
	printk("  RTC runs at 32768 Hz, 1 tick = 30.517578125 us\n");
	printk("  offset_ticks     = %u\n", offset_ticks);
	printk("  offset_remainder = %u ps (sub-tick precision)\n",
	       offset_remainder_ps);

	printk("\n[Step 4] First anchor point (absolute RTC tick)\n");
	printk("  initial_anchor = connect_end_rtc + offset_ticks\n");
	printk("                 = %u + %u = %u\n",
	       connect_end_rtc, offset_ticks,
	       anchor.initial_anchor_rtc);

	printk("\n[Step 5] connInterval in RTC ticks\n");
	printk("  interval = %u us\n", conn_interval_us);
	printk("  interval_ticks     = %u\n", interval_ticks);
	printk("  interval_remainder = %u ps\n", interval_remainder_ps);
	printk("  (remainder accumulates each event, carries +1 tick\n");
	printk("   when >= 1 tick period, preventing drift)\n");

	printk("\n[Step 6] Window Widening parameters\n");
	printk("  Master SCA: %d ppm\n", master_sca_ppm);
	printk("  Local SCA:  %d ppm\n", LOCAL_SCA_PPM);
	printk("  Combined:   %d ppm\n", combined_sca_ppm);
	printk("  WW periodic = ceil(%d * %d / 1000000) = %d us/event\n",
	       combined_sca_ppm, conn_interval_us, ww_periodic_us);

	printk("\n===== Anchor Sequence Preview =====\n");
	printk("  A0 (first)  : RTC tick %u\n", anchor.initial_anchor_rtc);
	uint32_t a1 = (anchor.initial_anchor_rtc + interval_ticks) &
		      HAL_TICKER_CNTR_MASK;
	uint32_t a2 = (a1 + interval_ticks) & HAL_TICKER_CNTR_MASK;
	printk("  A1 (A0+intv) : RTC tick %u (delta=%u ticks)\n",
	       a1, interval_ticks);
	printk("  A2 (A1+intv) : RTC tick %u (delta=%u ticks)\n",
	       a2, interval_ticks);
	printk("  (remainder carry may add +1 tick per event)\n");
	printk("=====================================\n\n");
}

/*===========================================================================
 * Channel Selection Algorithm #1
 *
 * BLE Core Spec Vol 6, Part B, 4.5.8.1:
 *   unmappedChannel = (lastUnmappedChannel + hopIncrement) mod 37
 *   if unmappedChannel is used → return it
 *   else remap: remappingIndex = unmappedChannel mod numUsedChannels
 *===========================================================================*/
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
