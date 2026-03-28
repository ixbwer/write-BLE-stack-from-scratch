/*
 * BLE PHY 软件 TIFS Demo — 链路层 PDU 构造与校验
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LL_PDU_H
#define LL_PDU_H

#include "ble_common.h"

/* PDU 构造 */
void build_adv_ind_pdu(void);
void build_scan_rsp_pdu(void);

/* SCAN_REQ 校验 */
bool validate_scan_req(const struct pdu_adv *rx_pdu);

#endif /* LL_PDU_H */
