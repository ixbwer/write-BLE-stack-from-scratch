/*
 * BLE PHY Data Channel Demo — 广播态: ADV_IND 发送与 CONNECT_IND 接收
 * (与 05_phy_anchor 相同)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "adv.h"
#include "hal_radio.h"
#include "ll_pdu.h"

#define ADV_TX_SPINS          100000U
#define ADV_SWITCH_WAIT_SPINS 100000U

static struct k_timer adv_timer;
static struct k_work adv_work;
static const uint8_t adv_default_scan_rsp_data[] = {
	0x11, 0x09,
	'Z', 'e', 'p', 'h', 'y', 'r', '-',
	'S', 't', 'a', 'c', 'k', '-', 'B', 'L', 'E'
};

void adv_ctx_init_default(struct adv_ctx *ctx)
{
	ctx->addr = adv_addr;
	ctx->data = adv_data;
	ctx->data_len = adv_data_len;
	ctx->adv_pdu = &pdu_adv_ind;
	ctx->scan_rsp_pdu = &pdu_scan_rsp;
}

void adv_build_pdus(const struct adv_ctx *ctx)
{
	build_adv_ind_pdu_with(ctx->adv_pdu, ctx->addr, ctx->data,
			       ctx->data_len);
	build_scan_rsp_pdu_with(ctx->scan_rsp_pdu, ctx->addr,
				adv_default_scan_rsp_data,
				sizeof(adv_default_scan_rsp_data));
}

static void adv_primary_tx_prepare(const struct adv_ctx *ctx)
{
	radio_irq_disable_all();

	radio_pkt_tx_set(ctx->adv_pdu);
	radio_status_reset();
	radio_shorts_ready_start_end_disable_set();

	ble_sw_switch_reset();
	radio_sw_switch_timer_start();
	radio_tx_enable();
}

static int adv_scannable_channel_once(const struct adv_ctx *ctx,
			      uint32_t channel)
{
	if (!radio_adv_channel_set(channel)) {
		return 0;
	}

	adv_primary_tx_prepare(ctx);

	if (!radio_wait_done(ADV_TX_SPINS)) {
		radio_ensure_disabled();
		return 0;
	}

	if (!radio_sw_switch_to_rx(&pdu_rx_buf, ADV_SWITCH_WAIT_SPINS)) {
		radio_ensure_disabled();
		return 0;
	}

	if (!radio_wait_address(ADV_RX_TIMEOUT_US * 16U)) {
		radio_ensure_disabled();
		return 0;
	}

	ble_sw_switch(false);

	if (!radio_wait_done(ADV_TX_SPINS)) {
		radio_ensure_disabled();
		return 0;
	}

	if (!radio_crc_is_valid() ||
	    !validate_scan_req_for(&pdu_rx_buf, ctx->addr)) {
		radio_ensure_disabled();
		return 0;
	}

	scan_req_count++;
	radio_pkt_tx_set(ctx->scan_rsp_pdu);
	radio_status_reset();

	if (!radio_wait_disabled(ADV_SWITCH_WAIT_SPINS, false)) {
		radio_ensure_disabled();
		return 0;
	}

	scan_rsp_count++;
	radio_ensure_disabled();
	return 1;
}

void adv_worker(struct k_work *work)
{
	ARG_UNUSED(work);
	struct adv_ctx ctx;

	static const uint32_t adv_channels[] = {37, 38, 39};
	adv_ctx_init_default(&ctx);
	adv_build_pdus(&ctx);

	for (int i = 0; i < 3; i++) {
		(void)adv_scannable_channel_once(&ctx, adv_channels[i]);
	}

	adv_event_count++;
}

void adv_timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_work_submit(&adv_work);
}

void adv_init(void)
{
	k_work_init(&adv_work, adv_worker);
	k_timer_init(&adv_timer, adv_timer_expiry, NULL);
}

void adv_start(void)
{
	k_timer_start(&adv_timer, K_NO_WAIT, K_MSEC(ADV_INTERVAL_MS));
}

static int adv_on_channel_ctx(const struct adv_ctx *ctx, uint32_t channel)
{
	if (!radio_adv_channel_set(channel)) {
		return 0;
	}
	adv_primary_tx_prepare(ctx);

	if (!radio_wait_done(ADV_TX_SPINS)) {
		radio_ensure_disabled();
		return 0;
	}

	if (!radio_sw_switch_to_rx(&pdu_adv_rx, ADV_SWITCH_WAIT_SPINS)) {
		radio_ensure_disabled();
		return 0;
	}

	bool rx_done = radio_wait_disabled(ADV_RX_TIMEOUT_US * 16U, true);

	uint32_t rx_end_rtc = nrf_rtc_counter_get(NRF_RTC0) & HAL_TICKER_CNTR_MASK;

	if (!rx_done) {
		return 0;
	}

	radio_sw_switch_cleanup();

	if (!radio_crc_is_valid()) {
		return 0;
	}

	if (validate_connect_ind_for(&pdu_adv_rx, ctx->addr)) {
		connect_end_rtc = rx_end_rtc;
		parse_connect_ind(&pdu_adv_rx);
		return 1;
	}

	return 0;
}

int adv_on_channel(uint32_t channel)
{
	struct adv_ctx ctx;

	adv_ctx_init_default(&ctx);
	adv_build_pdus(&ctx);

	return adv_on_channel_ctx(&ctx, channel);
}

int adv_event(void)
{
	struct adv_ctx ctx;
	static const uint32_t adv_channels[] = {37, 38, 39};

	adv_ctx_init_default(&ctx);
	adv_build_pdus(&ctx);

	for (int i = 0; i < 3; i++) {
		int ret = adv_on_channel_ctx(&ctx, adv_channels[i]);
		if (ret == 1) {
			return 1;
		}
	}

	adv_event_count++;
	if (adv_event_count <= 3 || adv_event_count % 50 == 0) {
		printk("[ADV] events: %u (waiting for connection...)\n",
		       adv_event_count);
	}
	return 0;
}
