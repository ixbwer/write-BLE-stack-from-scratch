/*
 * BLE PHY 软件 TIFS Demo — 链路层 PDU 构造与校验
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ll_pdu.h"

/*===========================================================================
 * AD 数据
 *===========================================================================*/
static const uint8_t adv_data[] = {
	0x02, 0x01, 0x06,
	0x09, 0x09, 'Z', 'e', 'p', 'h', 'y', 'r', 'S', 'W'
};
#define ADV_DATA_LEN sizeof(adv_data)

static const uint8_t scan_rsp_data[] = {
	0x11, 0x09, 'Z', 'e', 'p', 'h', 'y', 'r', 'S', 'W',
	             '-', 'S', 'c', 'a', 'n', 'R', 's', 'p',
};
#define SCAN_RSP_DATA_LEN sizeof(scan_rsp_data)

/*===========================================================================
 * 构造 PDU
 *===========================================================================*/
void build_adv_ind_pdu(void)
{
	memset(&pdu_adv_ind, 0, sizeof(pdu_adv_ind));
	pdu_adv_ind.type    = PDU_ADV_TYPE_ADV_IND;
	pdu_adv_ind.tx_addr = 1;
	pdu_adv_ind.rx_addr = 0;
	pdu_adv_ind.len     = BDADDR_SIZE + ADV_DATA_LEN;
	memcpy(pdu_adv_ind.adv_ind.addr, adv_addr, BDADDR_SIZE);
	memcpy(pdu_adv_ind.adv_ind.data, adv_data, ADV_DATA_LEN);
	printk("ADV_IND PDU built: type=0x%02X len=%d\n",
	       pdu_adv_ind.type, pdu_adv_ind.len);
}

void build_scan_rsp_pdu(void)
{
	memset(&pdu_scan_rsp, 0, sizeof(pdu_scan_rsp));
	pdu_scan_rsp.type    = PDU_ADV_TYPE_SCAN_RSP;
	pdu_scan_rsp.tx_addr = 1;
	pdu_scan_rsp.rx_addr = 0;
	pdu_scan_rsp.len     = BDADDR_SIZE + SCAN_RSP_DATA_LEN;
	memcpy(pdu_scan_rsp.scan_rsp.addr, adv_addr, BDADDR_SIZE);
	memcpy(pdu_scan_rsp.scan_rsp.data, scan_rsp_data, SCAN_RSP_DATA_LEN);
	printk("SCAN_RSP PDU built: type=0x%02X len=%d\n",
	       pdu_scan_rsp.type, pdu_scan_rsp.len);
}

/*===========================================================================
 * 验证 SCAN_REQ
 *===========================================================================*/
bool validate_scan_req(const struct pdu_adv *rx_pdu)
{
	if (rx_pdu->type != PDU_ADV_TYPE_SCAN_REQ) {
		return false;
	}
	if (rx_pdu->len != sizeof(struct pdu_adv_scan_req)) {
		return false;
	}
	if (memcmp(rx_pdu->scan_req.adv_addr, adv_addr, BDADDR_SIZE) != 0) {
		return false;
	}
	return true;
}
