/*
 * 单次 BLE 连接事件: 锚点 RX → RX/TX 交互循环
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "conn_event.h"
#include "hal_radio.h"
#include "ll_pdu.h"
#include "hci.h"
#include "l2cap.h"

#if BLE_FEATURE_SMP
#include "smp.h"
#include "crypto.h"
#endif
#if BLE_FEATURE_ATT
#include "att.h"
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * 处理收到的 RX PDU
 *
 * 流程:
 *   1. 若加密已启用, 先 CCM 解密 + MIC 校验          (13+ BLE_FEATURE_SMP)
 *   2. ARQ: 检查 NESN (对端确认了我们上一包吗?)
 *   3. ARQ: 检查 SN (这是新包还是重传?)
 *   4. 分发新包: LL Control → handle_ll_control()
 *                L2CAP/ACL  → hci_send_acl_data()     (09+ BLE_FEATURE_HCI)
 *   5. HCI ACL TX 桥接: Host 发来的数据放入 TX 队列   (09+)
 *
 * 返回 true 表示本包是新包 (non-duplicate)。
 * ═══════════════════════════════════════════════════════════════════════════ */
static bool process_rx_pdu(void)
{
#if BLE_FEATURE_SMP
	/* ★ 加密: 如果 RX 解密开启, 先解密再处理 */
	if (enc.rx_encrypted && pdu_data_rx.len > 0) {
		bool mic_ok = false;
		bool ok = crypto_ccm_decrypt(&enc.ccm_rx,
					     &pdu_data_rx,
					     ccm_scratch_rx,
					     &mic_ok);

		if (ok && mic_ok) {
			memcpy(&pdu_data_rx, ccm_scratch_rx,
			       2 + ((struct pdu_data *)ccm_scratch_rx)->len);
			enc.ccm_rx.counter++;
		} else if (ok && !mic_ok &&
			   enc.ccm_rx.counter == 0 &&
			   pdu_data_rx.ll_id == PDU_DATA_LLID_CTRL) {
			/* 计数器为 0 且是 LL Control PDU: 视为明文 */
			printk("[ENC] MIC fail at counter=0, treating as plaintext\n");
		} else {
			printk("[ENC] ★ MIC failure! Terminating connection.\n");
			conn_terminate_reason = 0x3D;
			conn_terminated = true;
			return false;
		}
	}
#endif /* BLE_FEATURE_SMP */

	/* ── ARQ: NESN → 确认对端收到了我们的上一包 ── */
	if (pdu_data_rx.nesn != tx_sn) {
		tx_sn = pdu_data_rx.nesn;

#if BLE_FEATURE_SMP
		if (enc.phase == ENC_PHASE_RSP_SENT) {
			enc.phase = ENC_PHASE_START_REQ_PEND;
			enc.rx_encrypted = true;
			printk("[ENC] LL_ENC_RSP acked → RX decryption ON\n");
		}
#endif
	}

	/* ── ARQ: SN → 判断是否是新包 ── */
	bool new_packet = false;

	if (pdu_data_rx.sn == rx_nesn) {
		new_packet = true;
		rx_nesn ^= 1;
	}

	tx_pdu_pending = false;

	/* ── 分发新包 ── */
	if (new_packet && pdu_data_rx.len > 0) {
		if (pdu_data_rx.ll_id == PDU_DATA_LLID_CTRL) {
			tx_pdu_pending = handle_ll_control(&pdu_data_rx);
		} else if (pdu_data_rx.ll_id == PDU_DATA_LLID_DATA_START ||
			   pdu_data_rx.ll_id == PDU_DATA_LLID_DATA_CONTINUE) {
			bool is_start = (pdu_data_rx.ll_id ==
					 PDU_DATA_LLID_DATA_START);
			hci_send_acl_data(pdu_data_rx.lldata,
					  pdu_data_rx.len,
					  is_start);
		}
	}

	/* ── HCI ACL TX 桥接: Host 发来的数据 → TX 队列 ── */
	if (!tx_pdu_pending && hci_acl_tx_pending()) {
		uint8_t acl_buf[HCI_ACL_MAX_DATA];
		uint16_t acl_len = hci_acl_tx_get(acl_buf, sizeof(acl_buf));

		if (acl_len > 0) {
			l2cap_tx_enqueue(acl_buf, acl_len);
			hci_send_num_completed_pkts(1);
		}
	}

	return new_packet;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 准备 TX PDU 并设置到 RADIO DMA
 *
 * 优先级:
 *   1. LL Control 响应 (由 handle_ll_control 构建, tx_pdu_pending=true)
 *   2. ★ LL_START_ENC_REQ (明文, 加密握手)          (13+ BLE_FEATURE_SMP)
 *   3. Slave 主动断连 TERMINATE_IND                  (11+ BLE_FEATURE_SLAVE_TERMINATE)
 *   4. Slave 连接参数更新 CONN_PARAM_REQ              (11+ BLE_FEATURE_SLAVE_CPR)
 *   5. L2CAP TX 队列 (HCI ACL 下行数据)              (09+)
 *   6. 空 PDU
 *
 * 返回 true 表示 Slave 还有更多数据 (MD=1)。
 * ═══════════════════════════════════════════════════════════════════════════ */
static bool prepare_tx_pdu(void)
{
	if (tx_pdu_pending) {
		tx_pdu_buf.nesn = rx_nesn;
		tx_pdu_buf.sn   = tx_sn;
		tx_pdu_buf.md   = (data_tx_q.count > 0) ? 1 : 0;
		goto encrypt_and_set;
	}

#if BLE_FEATURE_SMP
	/* ★ 加密握手: 发送 LL_START_ENC_REQ (明文!) */
	if (enc.phase == ENC_PHASE_START_REQ_PEND) {
		build_start_enc_req(&tx_pdu_buf);
		tx_pdu_pending = true;
		enc.phase = ENC_PHASE_WAIT_START_RSP;
		radio_pkt_tx_set(&tx_pdu_buf);
		return false;
	}
#endif

#if BLE_FEATURE_SLAVE_TERMINATE
	if (proc_slave_term.pending) {
		build_terminate_ind(&tx_pdu_buf, proc_slave_term.reason);
		tx_pdu_pending = true;
		conn_terminate_reason = proc_slave_term.reason;
		conn_terminated = true;
		goto encrypt_and_set;
	}
#endif

#if BLE_FEATURE_SLAVE_CPR
	if (proc_slave_cpr.pending && !proc_slave_cpr.sent) {
		build_conn_param_req(&tx_pdu_buf);
		tx_pdu_pending = true;
		proc_slave_cpr.sent = true;
		goto encrypt_and_set;
	}
#endif

	if (l2cap_tx_dequeue(&tx_pdu_buf)) {
		tx_pdu_buf.nesn = rx_nesn;
		tx_pdu_buf.sn   = tx_sn;
		tx_pdu_pending = true;
		goto encrypt_and_set;
	}

	build_empty_pdu(&tx_pdu_buf);
	tx_pdu_buf.md = 0;
	radio_pkt_tx_set(&tx_pdu_buf);
	return false;

encrypt_and_set:
#if BLE_FEATURE_SMP
	if (enc.tx_encrypted && tx_pdu_buf.len > 0) {
		crypto_ccm_encrypt(&enc.ccm_tx, &tx_pdu_buf, ccm_scratch_tx);
		enc.ccm_tx.counter++;
		radio_pkt_tx_set(ccm_scratch_tx);
	} else {
		radio_pkt_tx_set(&tx_pdu_buf);
	}
#else
	radio_pkt_tx_set(&tx_pdu_buf);
#endif
	return tx_pdu_buf.md;
}

/* ─── 等待 TX DISABLED 事件 ─── */
static void wait_tx_done(void)
{
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_END);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	(void)radio_wait_disabled(2000000U, true);
}

/* ─── 等待 Multi-PDU 续传 RX ─── */
static bool wait_continuation_rx(void)
{
	if (!radio_wait_disabled(100000U, true)) {
		return false;
	}
	if (!radio_is_done()) {
		return false;
	}
	if (!radio_crc_is_valid()) {
		return false;
	}
	return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * conn_event_run — 执行一次完整的连接事件
 *
 * 整体流程:
 *   1. 将 RADIO RXEN 安排在 ticks_at_start (由 RTC0 compare 触发)
 *   2. 等待首次 RX (锚点接收) — 超时或 CRC 错误则返回 false
 *   3. 处理首次 RX + 发送首次 TX (通过 PPI 自动 RX→TX 切换)
 *   4. Multi-PDU: 若双方 MD=1 则在同一事件内继续 RX→TX (10+)
 *   5. 更新统计计数器
 * ═══════════════════════════════════════════════════════════════════════════ */
bool conn_event_run(uint32_t ticks_at_start, uint32_t remainder_ps,
		    uint32_t hcto_add)
{
	uint32_t ahead = (ticks_at_start - NRF_RTC0->COUNTER) &
			 HAL_TICKER_CNTR_MASK;

	if (ahead > (HAL_TICKER_CNTR_MASK >> 1) ||
	    ahead < RTC0_CMP_OFFSET_MIN) {
		(void)chan_sel_1();
		conn_event_rx_timeout++;
		conn_event_counter++;
		return false;
	}

	uint8_t chan = chan_sel_1();

	data_chan_set(chan);

	NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;
	radio_status_reset();
	radio_tmr_status_reset();

	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;

	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN) | BIT(PPI_CH_TXEN));

	/*
	 * 预配置双向 CC 值 (仅需一次, PPI 每次 END 都会清零 TIMER1):
	 *   CC[TXEN]: RX END → TXEN  (RX→TX 切换)
	 *   CC[RXEN]: TX END → RXEN  (TX→RX 切换, Multi-PDU 时使用)
	 */
	nrf_timer_cc_set(NRF_TIMER1, (nrf_timer_cc_channel_t)CC_IDX_TXEN,
			 T_IFS_US - TX_RAMP_US - RX_CHAIN_DELAY_1M_US);
	nrf_timer_cc_set(NRF_TIMER1, (nrf_timer_cc_channel_t)CC_IDX_RXEN,
			 TX_TO_RX_CC_US);

	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_START);

	radio_pkt_rx_set(&pdu_data_rx);

	nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_STOP);
	NRF_RTC0->EVENTS_COMPARE[2] = 0;

	uint32_t remainder_us = radio_tmr_start(0, ticks_at_start,
						remainder_ps);

	radio_tmr_aa_capture();
	radio_tmr_aa_save(0);

	uint32_t hcto = remainder_us + hcto_add +
			radio_rx_ready_delay_get(0, 0) +
			ADDR_US_1M +
			radio_rx_chain_delay_get(0, 0);
	radio_tmr_hcto_configure(hcto);
	radio_tmr_end_capture();

	/* ── 等待首次 RX (锚点) ── */
	if (!radio_wait_disabled(5000000U, true)) {
		conn_event_rx_timeout++;
		conn_event_counter++;
		return false;
	}

	if (!radio_is_done()) {
		radio_ensure_disabled();
		conn_event_rx_timeout++;
		conn_event_counter++;
		return false;
	}

	if (!radio_crc_is_valid()) {
		radio_ensure_disabled();
		conn_event_rx_crc_err++;
		conn_event_counter++;
		return false;
	}

	conn_event_rx_ok++;

	last_rx_aa_us    = radio_tmr_aa_get();
	last_rx_ready_us = radio_tmr_ready_get();

	/* 清除 CC 比较事件, 准备 RX→TX 切换 */
	NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN] = 0;
	(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN];
	NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN] = 0;
	(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN];

	/* ── 首次 RX 处理 + 首次 TX 发送 ── */
	process_rx_pdu();
	bool slave_has_more = prepare_tx_pdu();

	nrf_ppi_channels_enable(NRF_PPI, BIT(PPI_CH_TXEN));
	wait_tx_done();
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_TXEN));

	if (tx_pdu_pending) {
		tx_sn ^= 1;
	}

	/* ── Multi-PDU 续传循环 ── */
	/*
	 * BLE Core Spec Vol 6, Part B, §4.5.3:
	 * 连接事件在双方最新 PDU 的 MD=0 时关闭。
	 * 若任一方 MD=1, 可在同一事件内继续 RX→TX 交互。
	 * (BLE_FEATURE_MULTI_PDU=0 时 while 条件始终为 false, 编译器会优化掉)
	 */
	bool     master_md          = pdu_data_rx.md;
	uint32_t exchanges_this_event = 1;

	while (BLE_FEATURE_MULTI_PDU &&
	       (master_md || slave_has_more) &&
	       exchanges_this_event < MAX_EXCHANGES_PER_EVENT &&
	       !conn_terminated) {

		NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN] = 0;
		(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN];

		NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
				    RADIO_SHORTS_END_DISABLE_Msk;

		radio_pkt_rx_set(&pdu_data_rx);
		nrf_ppi_channels_enable(NRF_PPI, BIT(PPI_CH_RXEN));

		bool rx_ok = wait_continuation_rx();

		nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN));

		if (!rx_ok) {
			break;
		}

		conn_event_rx_ok++;

		NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN] = 0;
		(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN];
		NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN] = 0;
		(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN];

		process_rx_pdu();
		master_md = pdu_data_rx.md;

		slave_has_more = prepare_tx_pdu();

		nrf_ppi_channels_enable(NRF_PPI, BIT(PPI_CH_TXEN));
		wait_tx_done();
		nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_TXEN));

		if (tx_pdu_pending) {
			tx_sn ^= 1;
		}

		exchanges_this_event++;
	}

	NRF_RADIO->SHORTS = 0;
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN) | BIT(PPI_CH_TXEN));

	/* ── 统计 ── */
	multi_pdu_total_exchanges += exchanges_this_event;
	if (exchanges_this_event > 1) {
		multi_pdu_events_extended++;
		if (exchanges_this_event > multi_pdu_max_in_event) {
			multi_pdu_max_in_event = exchanges_this_event;
		}
	}

	conn_event_counter++;
	return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 会话级辅助 (供 conn_loop() 直接使用的高层接口)
 * ═══════════════════════════════════════════════════════════════════════════ */

void conn_event_reset(void)
{
	tx_sn                     = 0;
	rx_nesn                   = 0;
	conn_event_counter        = 0;
	last_unmapped_chan        = 0;
	tx_pdu_pending            = false;
	conn_event_rx_ok          = 0;
	conn_event_rx_timeout     = 0;
	conn_event_rx_crc_err     = 0;
	multi_pdu_total_exchanges = 0;
	multi_pdu_events_extended = 0;
	multi_pdu_max_in_event    = 0;

	/*
	 * LL procedure 请求槽、L2CAP CPURQ、加密上下文 — 每次进入连接时必须
	 * 清零, 保证跨连接不残留旧状态。无论 BLE_FEATURE_LL_PROCEDURE 是否开启
	 * 都需要复位, 因为这些变量在 BSS 区始终存在 (由 compat_globals.c 提供
	 * __weak 定义)。
	 */
	memset(&proc_conn_update, 0, sizeof(proc_conn_update));
	memset(&proc_chan_map,    0, sizeof(proc_chan_map));
	memset(&proc_slave_term,  0, sizeof(proc_slave_term));
	memset(&proc_slave_cpr,   0, sizeof(proc_slave_cpr));
	memset(&l2cap_cpurq,      0, sizeof(l2cap_cpurq));
	l2cap_cpurq.identifier = 1;
	memset(&enc, 0, sizeof(enc));

	/* 上层栈重置: 未启用的特性通过编译期剥离或弱符号退化为空操作 */
#if BLE_FEATURE_SMP
	smp_init();
#endif
#if BLE_FEATURE_ATT
	att_init();
#endif
	l2cap_init();

	radio_configure_conn();

	/*
	 * adv.c 期间可能把 PPI_CH_RXEN/TXEN 的端点改到 TASKS_TXEN/RXEN 之外,
	 * 这里在进入连接态之前恢复到连接事件使用的配置。
	 */
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN) | BIT(PPI_CH_TXEN));
	nrf_ppi_channel_endpoint_setup(
		NRF_PPI, (nrf_ppi_channel_t)PPI_CH_TXEN,
		nrf_timer_event_address_get(NRF_TIMER1,
					    nrf_timer_compare_event_get(CC_IDX_TXEN)),
		nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_TXEN));
	nrf_ppi_channel_endpoint_setup(
		NRF_PPI, (nrf_ppi_channel_t)PPI_CH_RXEN,
		nrf_timer_event_address_get(NRF_TIMER1,
					    nrf_timer_compare_event_get(CC_IDX_RXEN)),
		nrf_radio_task_address_get(NRF_RADIO, NRF_RADIO_TASK_RXEN));
}

void conn_event_teardown(void)
{
	NRF_RADIO->SHORTS   = 0;
	NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;
}

void conn_event_summary(void)
{
	printk("\n========== Connection Summary ==========\n");
	printk("  Total events:       %u\n", conn_event_counter);
	printk("  RX OK:              %u\n", conn_event_rx_ok);
	printk("  RX timeout:         %u\n", conn_event_rx_timeout);
	printk("  RX CRC error:       %u\n", conn_event_rx_crc_err);
	printk("  Disconnect reason:  0x%02X\n", conn_terminate_reason);
}
