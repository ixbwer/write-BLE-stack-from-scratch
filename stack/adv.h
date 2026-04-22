/*
 * BLE PHY Data Channel Demo — 广播态: ADV_IND 发送与 CONNECT_IND 接收
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ADV_H
#define ADV_H

#include "ble_common.h"

struct adv_ctx {
	const uint8_t *addr;
	const uint8_t *data;
	uint8_t data_len;
	struct pdu_adv *adv_pdu;
	struct pdu_adv *scan_rsp_pdu;
};

void adv_ctx_init_default(struct adv_ctx *ctx);
void adv_build_pdus(const struct adv_ctx *ctx);

void adv_worker(struct k_work *work);
void adv_timer_expiry(struct k_timer *timer);
void adv_init(void);
void adv_start(void);

int adv_on_channel(uint32_t channel);
int adv_event(void);

#endif /* ADV_H */
