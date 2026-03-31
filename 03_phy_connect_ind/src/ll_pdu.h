/*
 * BLE PHY CONNECT_IND Demo — 链路层 PDU 构造与校验
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

/* CONNECT_IND */
bool validate_connect_ind(const struct pdu_adv *rx_pdu);
void parse_connect_ind(const struct pdu_adv *rx_pdu);
void print_connect_ind(void);

#endif /* LL_PDU_H */
