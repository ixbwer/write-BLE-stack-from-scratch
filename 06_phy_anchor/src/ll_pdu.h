/*
 * BLE PHY Anchor Demo — 链路层: PDU 构造、LL Control、信道选择
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LL_PDU_H
#define LL_PDU_H

#include "ble_common.h"

/* 辅助函数 */
uint8_t count_ones(const uint8_t *data, uint8_t len);

/* ADV PDU */
void build_adv_ind_pdu(void);

/* Data PDU */
void build_empty_pdu(struct pdu_data *pdu);
void build_version_ind(struct pdu_data *pdu);
void build_feature_rsp(struct pdu_data *pdu);
void build_unknown_rsp(struct pdu_data *pdu, uint8_t unknown_type);

/* LL Control */
bool handle_ll_control(const struct pdu_data *rx_pdu);

/* CONNECT_IND */
bool validate_connect_ind(const struct pdu_adv *rx_pdu);
void parse_connect_ind(const struct pdu_adv *rx_pdu);
void print_connect_ind(void);
void print_anchor_calculation(uint32_t conn_offset_us,
			      uint32_t offset_ticks,
			      uint32_t offset_remainder_ps,
			      uint32_t interval_ticks,
			      uint32_t interval_remainder_ps,
			      uint16_t master_sca_ppm,
			      uint32_t combined_sca_ppm,
			      uint32_t ww_periodic_us,
			      uint32_t conn_interval_us);

/* Channel Selection Algorithm #1 */
uint8_t chan_sel_1(void);

#endif /* LL_PDU_H */
