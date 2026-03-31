/*
 * BLE PHY Data Channel Demo — 链路层: PDU 构造、LL Control、信道选择
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LL_PDU_H
#define LL_PDU_H

#include "ble_common.h"

uint8_t count_ones(const uint8_t *data, uint8_t len);

/* ADV PDU */
void build_adv_ind_pdu(void);

/* Data PDU */
void build_empty_pdu(struct pdu_data *pdu);
void build_version_ind(struct pdu_data *pdu);
void build_feature_rsp(struct pdu_data *pdu);
void build_unknown_rsp(struct pdu_data *pdu, uint8_t unknown_type);
void build_length_rsp(struct pdu_data *pdu);

/* LL Control */
bool handle_ll_control(const struct pdu_data *rx_pdu);

/* CONNECT_IND */
bool validate_connect_ind(const struct pdu_adv *rx_pdu);
void parse_connect_ind(const struct pdu_adv *rx_pdu);
void print_connect_ind(void);

/* Channel Selection Algorithm #1 */
uint8_t chan_sel_1(void);

#endif /* LL_PDU_H */
