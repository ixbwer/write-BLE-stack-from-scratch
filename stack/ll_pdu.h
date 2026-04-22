/*
 * BLE PHY Encryption Demo — 链路层: PDU 构造、LL Control、信道选择
 *
 * 相比 12_phy_l2cap_sig 的变化:
 *   1. LL_ENC_REQ 从拒绝改为正确处理 (加密握手)
 *   2. 新增 build_enc_rsp(), build_start_enc_req(), build_start_enc_rsp()
 *   3. 新增 LL_START_ENC_RSP 处理
 *   4. feature_rsp 声明支持 LE Encryption
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LL_PDU_H
#define LL_PDU_H

#include "ble_common.h"

uint8_t count_ones(const uint8_t *data, uint8_t len);

/* ADV PDU */
void build_adv_ind_pdu_with(struct pdu_adv *pdu, const uint8_t *addr,
			    const uint8_t *data, uint8_t data_len);
void build_adv_ind_pdu(void);
void build_scan_rsp_pdu_with(struct pdu_adv *pdu, const uint8_t *addr,
			     const uint8_t *data, uint8_t data_len);
void build_scan_rsp_pdu(void);
bool validate_connect_ind_for(const struct pdu_adv *rx_pdu,
			      const uint8_t *addr);
bool validate_scan_req_for(const struct pdu_adv *rx_pdu, const uint8_t *addr);
bool validate_scan_req(const struct pdu_adv *rx_pdu);

/* Data PDU */
void build_empty_pdu(struct pdu_data *pdu);
void build_version_ind(struct pdu_data *pdu);
void build_feature_rsp(struct pdu_data *pdu);
void build_unknown_rsp(struct pdu_data *pdu, uint8_t unknown_type);
void build_length_rsp(struct pdu_data *pdu);

/* LL Control */
bool handle_ll_control(const struct pdu_data *rx_pdu);

/* Slave-initiated LL Control PDUs */
void build_terminate_ind(struct pdu_data *pdu, uint8_t reason);
void build_reject_ext_ind(struct pdu_data *pdu, uint8_t reject_opcode,
			 uint8_t error_code);
void build_conn_param_req(struct pdu_data *pdu);

/* ★ Encryption LL Control PDUs */
void build_enc_rsp(struct pdu_data *pdu);
void build_start_enc_req(struct pdu_data *pdu);
void build_start_enc_rsp(struct pdu_data *pdu);

/* CONNECT_IND */
bool validate_connect_ind(const struct pdu_adv *rx_pdu);
void parse_connect_ind(const struct pdu_adv *rx_pdu);
void print_connect_ind(void);

/* Channel Selection Algorithm #1 */
uint8_t chan_sel_1(void);

/* Channel remap helper (used when channel map changes at Instant) */
uint8_t remap_channel(uint8_t unmapped, const uint8_t *chan_map,
		      uint8_t chan_count);

#endif /* LL_PDU_H */
