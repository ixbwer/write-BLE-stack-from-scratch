/*
 * BLE PHY 软件 TIFS Demo — 广播状态机
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "adv.h"
#include "hal_radio.h"
#include "ll_pdu.h"

/*===========================================================================
 * 调度器组件
 *===========================================================================*/
static struct k_timer adv_timer;
static struct k_work  adv_work;

/*===========================================================================
 * 在指定信道上执行软件 TIFS 的 ADV_IND + SCAN_REQ + SCAN_RSP
 *
 * ★ 核心差异 vs phy_tifs_hw:
 *
 *   HW TIFS:
 *     NRF_RADIO->TIFS = 150;
 *     SHORTS = ... | DISABLED_RXEN;  // 硬件自动延时后切换
 *
 *   SW TIFS (本 Demo):
 *     TIMER1->CC[x] = TIFS - ramp_delay;  // 设置定时器
 *     PPI: TIMER1 CC[x] → RADIO TXEN/RXEN; // PPI 自动触发
 *     SHORTS = READY_START | END_DISABLE;   // 无 DISABLED→EN!
 *
 * 对比真实 Controller 的 sw_switch() 函数:
 *
 *   真实 Controller:                     本 Demo:
 *   ─────────────────────                ──────────────────
 *   nrf_timer_cc_set(SW_SWITCH_TIMER,    TIMER1->CC[0] = 110
 *     SW_SWITCH_TIMER_EVTS_COMP(toggle),
 *     tifs - delay);
 *
 *   hal_radio_rxen_on_sw_switch(cc,ppi)  PPI CH1: CC[0]→RXEN
 *   hal_radio_txen_on_sw_switch(cc,ppi)  PPI CH2: CC[1]→TXEN
 *
 *   sw_tifs_toggle ^= 1;                 (简化: 不使用 toggle)
 *
 * 关键区别: 真实 Controller 使用 toggle (交替 CC[0]/CC[1]) 和
 * PPI Groups 来避免陈旧 COMPARE 事件。我们通过在每次转换前
 * 清除 EVENTS_COMPARE 来达到同样的效果。
 *===========================================================================*/
static void adv_on_channel(uint32_t channel)
{
	uint32_t freq;

	switch (channel) {
	case 37: freq = 2;  break;
	case 38: freq = 26; break;
	case 39: freq = 80; break;
	default: return;
	}

	radio_freq_chan_set(freq);
	radio_whiten_iv_set(channel);
	NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;

	/*==============================================================
	 * 第 1 阶段: TX ADV_IND
	 *
	 * 注意: PPI_CH_RXEN 在 TX 期间保持禁用!
	 * Timer + PPI_CH_TIMER_CLEAR 在 TX END 时清零 Timer。
	 * TX END 后才启用 PPI_CH_RXEN, 此时 Timer 从 0 开始计数,
	 * 110μs 后 RXEN 触发, Radio 进入 RX。
	 *==============================================================*/

	radio_pkt_tx_set(&pdu_adv_ind);

	radio_status_reset();
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_ADDRESS = 0;

	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;

	/* 设置 CC 值 (TX→RX 和 RX→TX 阶段分别使用)
	 *
	 * CC[0] (TX→RX): ADV_IND TX END 后切 RX
	 *   TX END 事件与空口最后 bit 对齐 (无 chain delay)
	 *   CC[0] = T_IFS - RX_RAMP = 150 - 40 = 110μs
	 *
	 * CC[1] (RX→TX): SCAN_REQ RX END 后切 TX
	 *   RX END 事件比空口最后 bit 晚 RX_CHAIN_DELAY ≈ 10μs
	 *   必须补偿: CC[1] = T_IFS - TX_RAMP - RX_CHAIN_DELAY
	 *                    = 150 - 40 - 10 = 100μs
	 */
	nrf_timer_cc_set(NRF_TIMER1, (nrf_timer_cc_channel_t)CC_IDX_RXEN,
			 T_IFS_US - RX_RAMP_US);                       /* 110μs */
	nrf_timer_cc_set(NRF_TIMER1, (nrf_timer_cc_channel_t)CC_IDX_TXEN,
			 T_IFS_US - TX_RAMP_US - RX_CHAIN_DELAY_US);   /* 100μs */

	/* 确保 PPI_CH_RXEN 和 PPI_CH_TXEN 都禁用 (TX 期间不需要自动切换) */
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN) | BIT(PPI_CH_TXEN));

	/* 启动 Timer (PPI_CH_TIMER_CLEAR 始终启用: END→Timer CLEAR) */
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_START);

	/* 开始 TX */
	NRF_RADIO->TASKS_TXEN = 1;

	/* 等待 TX END (ADV_IND 发送完毕)
	 * TX END 时: PPI_CH_TIMER_CLEAR 自动清零 Timer,
	 *           SHORTS END→DISABLE 使 Radio 进入 DISABLED。 */
	while (NRF_RADIO->EVENTS_END == 0) {
	}
	NRF_RADIO->EVENTS_END = 0;

	/*==============================================================
	 * 第 1.5 阶段: TX→RX 转换 (SW TIFS)
	 *
	 * TX END 发生时:
	 *   - PPI_CH_TIMER_CLEAR: Timer 清零到 0, 继续计数
	 *   - SHORTS: Radio END→DISABLE → DISABLED
	 *
	 * 现在启用 PPI_CH_RXEN:
	 *   Timer 从 0 计数, 在 CC[0]=110μs 时触发 RXEN
	 *   Radio DISABLED→RXRU (40μs)→RX, 总计 T_IFS=150μs
	 *
	 * ★ 关键: 先清除 EVENTS_COMPARE (带读回), 再启用 PPI
	 *   TX 期间 Timer 经过了 CC[0]=110μs (EVENTS_COMPARE[0]=1),
	 *   若不清除, PPI 启用后不会看到新的 0→1 边沿。
	 *==============================================================*/

	/* 清除 COMPARE 事件 (TX 期间的陈旧事件), 写后读回确保 APB 生效 */
	NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN] = 0;
	(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN];
	NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN] = 0;
	(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN];

	/* 切换 PACKETPTR 到 RX 缓冲区 */
	radio_pkt_rx_set(&pdu_rx_buf);

	/* 现在启用 PPI_CH_RXEN: Timer 正在从 TX END 清零后计数,
	 * 约 110μs 后 CC[0] 触发 RXEN */
	nrf_ppi_channels_enable(NRF_PPI, BIT(PPI_CH_RXEN));

	/* 等待 TX DISABLED */
	while (NRF_RADIO->EVENTS_DISABLED == 0) {
	}
	NRF_RADIO->EVENTS_DISABLED = 0;

	/*==============================================================
	 * 第 2 阶段: RX 等待 SCAN_REQ
	 *
	 * 此时: TIMER1 从 TX END 清零后正在计数。
	 *   - t = 0:     TX END, PPI 清零 Timer
	 *   - t ≈ 2μs:  EVENTS_COMPARE 清除 + PPI RXEN 启用
	 *   - t = 110μs: CC[0] 匹配 → PPI CH1 → RXEN 触发
	 *   - t = 150μs: RX READY → START, Radio 开始监听
	 *==============================================================*/

	/* ★ 必须清除 TX 期间设置的 ADDRESS 事件!
	 * EVENTS_ADDRESS 在 TX 时被硬件置 1 (Access Address 发送到空中时),
	 * 若不清除, 下方 RX 等待循环会立即退出, 永远收不到 SCAN_REQ。 */
	NRF_RADIO->EVENTS_ADDRESS = 0;

	/* 等待 ADDRESS 事件 (AA 匹配, 有扫描器在发包) */
	uint32_t timeout = RX_TIMEOUT_US * 16;
	while (NRF_RADIO->EVENTS_ADDRESS == 0 && timeout > 0) {
		timeout--;
	}

	if (NRF_RADIO->EVENTS_ADDRESS == 0) {
		/* 超时: 无 SCAN_REQ */
		radio_ensure_disabled();
		return;
	}

	/* ADDRESS 匹配!
	 *
	 * 现在 CC[0] 已经完成了使命 (RXEN 已触发), 可以安全禁用 PPI_CH_RXEN。
	 * 同时启用 PPI_CH_TXEN: CC[1]=100μs 将在 RX END 后触发 TXEN → SCAN_RSP。
	 *
	 * ★ FIX: CC[1] 已在 TX 开始前设置好 (= T_IFS - TX_RAMP - RX_CHAIN_DELAY = 100μs)。
	 *   不能在此处设置 CC[1]! 因为 ADDRESS 触发时 Timer ≈ 190μs > 100μs,
	 *   CC[1] 已被过去, 下次匹配要等到 Timer 溢出 (65535μs 后)。
	 *   CC[1] 必须在 RX END 时 PPI 清零 Timer 之后生效, 而 Timer 的零点是 RX END。
	 *   我们在 TX 前设置好的 CC[1]=100, 在 RX END + 100μs 时自然触发。
	 *
	 * 对应真实 Controller:
	 *   sw_switch(SW_SWITCH_RX, SW_SWITCH_TX, phy_rx, ...);
	 *   → hal_radio_txen_on_sw_switch(cc, ppi)  (CC 已由 radio_tmr_tifs_set 设置)
	 */
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN));
	nrf_ppi_channels_enable(NRF_PPI, BIT(PPI_CH_TXEN));

	/* 等待 RX END (SCAN_REQ 接收完毕)
	 * SHORTS: END→DISABLE, PPI: END→TIMER CLEAR */
	while (NRF_RADIO->EVENTS_END == 0) {
	}
	NRF_RADIO->EVENTS_END = 0;

	/* RX END 后:
	 * - SHORTS: END→DISABLE → Radio DISABLED
	 * - PPI CH0: END → TIMER CLEAR → Timer 从 0 开始计数
	 * - t = 110μs 时: CC[1] → PPI CH2 (TXEN 已启用) → TXEN 触发 → TX SCAN_RSP
	 *
	 * ★ 必须清除两个 CC 事件!
	 * RX 期间 Timer 已计过 CC[0] 和 CC[1] (均为 110μs, RX 过 200μs 时已过)。
	 * 不清除的话 CC[1] 无法产生新的 0→1 边沿, PPI 不会触发 TXEN。
	 *
	 * ★ 写后读回确保 APB 写入生效 */
	NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN] = 0;
	(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN];
	NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN] = 0;
	(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN];

	/* 验证 SCAN_REQ
	 * 此时距 RX END ~5μs, 距 PPI 触发 TXEN 还有 ~105μs */
	if (!radio_crc_is_valid() || !validate_scan_req(&pdu_rx_buf)) {
		/* 无效: 禁用 TXEN PPI, 取消 TX 转换 */
		radio_ensure_disabled();
		return;
	}

	scan_req_count++;

	/*==============================================================
	 * 第 3 阶段: TX SCAN_RSP (由 SW TIFS PPI 自动触发)
	 *
	 * 时序 (从 RX END 开始, t=0):
	 *   t = 0:     RX END, PPI 清零 Timer → Timer 从 0 计数
	 *   t ≈ 1μs:  RX DISABLED (SHORTS END→DISABLE)
	 *   t ≈ 5μs:  验证 SCAN_REQ 完成 (我们在这里)
	 *   t = 100μs: CC[1] 匹配 → PPI CH2 → TXEN (Radio 从 DISABLED 进入 TXRU)
	 *   t = 140μs: TX READY → (SHORTS READY→START) → TX 开始
	 *   t ≈ 540μs: TX END → DISABLED
	 *
	 * 空口 T_IFS = 100 + 40 + RX_CHAIN_DELAY(10) = 150μs ✓
	 *
	 * 我们需要在 t=100μs 前完成: radio_pkt_tx_set + SHORTS 设置 + 清除事件。
	 * 有 ~95μs 的充足时间。
	 *==============================================================*/

	radio_pkt_tx_set(&pdu_scan_rsp);

	/* SHORTS: TX 完成后 DISABLE */
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;

	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;  /* ★ 必须清除: RX SHORTS 产生的 DISABLED 事件 */

	/* 等待 PPI 自动触发 TXEN → TX 完成后 DISABLED */
	while (NRF_RADIO->EVENTS_DISABLED == 0) {
	}

	/* 清理: 禁用 TXEN PPI */
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_TXEN));
	NRF_RADIO->SHORTS = 0;

	scan_rsp_count++;
}

/*===========================================================================
 * 广播工作处理函数
 *===========================================================================*/
void adv_worker(struct k_work *work)
{
	ARG_UNUSED(work);

	adv_on_channel(37);
	adv_on_channel(38);
	adv_on_channel(39);

	adv_event_count++;

	if (adv_event_count <= 3 || adv_event_count % 10 == 0) {
		printk("[SW-TIFS] events: %u | SCAN_REQ rx: %u | SCAN_RSP tx: %u\n",
		       adv_event_count, scan_req_count, scan_rsp_count);
	}
}

/*===========================================================================
 * 定时器回调
 *===========================================================================*/
void adv_timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_work_submit(&adv_work);
}

/*===========================================================================
 * 初始化调度器组件
 *===========================================================================*/
void adv_init(void)
{
	k_work_init(&adv_work, adv_worker);
	k_timer_init(&adv_timer, adv_timer_expiry, NULL);
}

/*===========================================================================
 * 启动广播定时器
 *===========================================================================*/
void adv_start(void)
{
	k_timer_start(&adv_timer, K_NO_WAIT, K_MSEC(ADV_INTERVAL_MS));
}
