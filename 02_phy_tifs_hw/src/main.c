/*
 * BLE PHY 全硬件 TIFS (HW TIFS) Demo
 *
 * 基于 phy_adv_scan, 本 Demo 的关键改进:
 *
 *   phy_adv_scan 中, TX→RX 使用硬件 TIFS, 但 RX→TX 使用 k_busy_wait 近似延时。
 *   本 Demo 让 TX→RX 和 RX→TX 都使用 NRF_RADIO->TIFS 寄存器 + SHORTS 实现
 *   全硬件精确 T_IFS = 150 μs。
 *
 * 硬件 TIFS 原理 (CONFIG_BT_CTLR_TIFS_HW 路径):
 *
 *   NRF_RADIO->TIFS = 150;
 *
 *   当配合 SHORTS 使用时:
 *     - END → DISABLE + DISABLED → RXEN: TX 完成后精确 150μs 开始 RX
 *     - END → DISABLE + DISABLED → TXEN: RX 完成后精确 150μs 开始 TX
 *
 *   硬件内部自动计算 ramp-up 时间, 保证:
 *     从上一包最后一 bit on air → 下一包第一 bit on air = TIFS 微秒
 *
 * 对比 radio_tmr_tifs_set() 的两种实现:
 *
 *   ┌──────────────────────────────────────────────────────────┐
 *   │ #if defined(CONFIG_BT_CTLR_TIFS_HW)                     │
 *   │     NRF_RADIO->TIFS = tifs;          ← 本 Demo 使用     │
 *   │ #else                                                    │
 *   │     nrf_timer_cc_set(SW_SWITCH_TIMER, ...);  ← SW Demo  │
 *   │ #endif                                                   │
 *   └──────────────────────────────────────────────────────────┘
 *
 * 完整工作流程:
 *
 *   ┌─────────┐  HW TIFS=150μs  ┌──────────┐  HW TIFS=150μs  ┌──────────┐
 *   │ ADV_IND ├────────────────►│ SCAN_REQ ├────────────────►│ SCAN_RSP │
 *   │  (TX)   │  DISABLED→RXEN │   (RX)   │  DISABLED→TXEN │   (TX)   │
 *   └─────────┘                 └──────────┘                 └──────────┘
 *     SHORTS 自动切换            SHORTS 自动切换              SHORTS 自动切换
 *
 * 对比 phy_adv_scan:
 *
 *   ┌────────────────┬───────────────────┬──────────────────────┐
 *   │  转换阶段      │  phy_adv_scan     │  本 Demo (phy_tifs_hw) │
 *   │────────────────│───────────────────│──────────────────────│
 *   │  TX → RX       │  HW TIFS + SHORTS │  HW TIFS + SHORTS    │
 *   │  RX → TX       │  k_busy_wait 近似 │  HW TIFS + SHORTS    │
 *   └────────────────┴───────────────────┴──────────────────────┘
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <soc.h>
#include <hal/nrf_clock.h>

/* ---- Controller HAL 层头文件 ---- */
#include "hal/ccm.h"
#include "hal/radio.h"

/* ---- Controller PDU 结构体定义 ---- */
#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

/*===========================================================================
 * 配置参数
 *===========================================================================*/

#define ADV_INTERVAL_MS  100

/* T_IFS: BLE 规范定义的帧间间隔 = 150 μs */
#define T_IFS_US         150

/* RX 等待超时 (μs): 等待 ADDRESS 事件的超时 */
#define RX_TIMEOUT_US    500

/* 广播设备名称 */
#define ADV_NAME         "ZephyrHW"

static const uint8_t adv_addr[BDADDR_SIZE] = {
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

/*===========================================================================
 * AD 数据
 *===========================================================================*/
static const uint8_t adv_data[] = {
	0x02, 0x01, 0x06,
	0x09, 0x09, 'Z', 'e', 'p', 'h', 'y', 'r', 'H', 'W'
};
#define ADV_DATA_LEN sizeof(adv_data)

static const uint8_t scan_rsp_data[] = {
	0x11, 0x09, 'Z', 'e', 'p', 'h', 'y', 'r', 'H', 'W',
	             '-', 'S', 'c', 'a', 'n', 'R', 's', 'p',
};
#define SCAN_RSP_DATA_LEN sizeof(scan_rsp_data)

/*===========================================================================
 * PDU 缓冲区
 *===========================================================================*/
static struct pdu_adv pdu_adv_ind __aligned(4);
static struct pdu_adv pdu_scan_rsp __aligned(4);
static struct pdu_adv pdu_rx_buf __aligned(4);

/*===========================================================================
 * 调度器组件
 *===========================================================================*/
static struct k_timer adv_timer;
static struct k_work  adv_work;

static volatile uint32_t adv_event_count;
static volatile uint32_t scan_req_count;
static volatile uint32_t scan_rsp_count;

/*===========================================================================
 * 构造 PDU
 *===========================================================================*/
static void build_adv_ind_pdu(void)
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

static void build_scan_rsp_pdu(void)
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
 * 启动 HFCLK
 *===========================================================================*/
static void hfclk_start(void)
{
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
	}
	printk("HFCLK started\n");
}

/*===========================================================================
 * 使用 HAL 接口配置 RADIO
 *===========================================================================*/
static void radio_configure(void)
{
	radio_phy_set(0, 0);
	radio_tx_power_set(0);

	radio_pkt_configure(RADIO_PKT_CONF_LENGTH_8BIT,
			    PDU_AC_LEG_PAYLOAD_SIZE_MAX,
			    RADIO_PKT_CONF_FLAGS(
				    RADIO_PKT_CONF_PDU_TYPE_AC,
				    RADIO_PKT_CONF_PHY_LEGACY,
				    RADIO_PKT_CONF_CTE_DISABLED));

	uint32_t aa = sys_cpu_to_le32(PDU_AC_ACCESS_ADDR);
	radio_aa_set((const uint8_t *)&aa);
	radio_crc_configure(PDU_CRC_POLYNOMIAL, PDU_AC_CRC_IV);

	/* ★ 核心: 设置硬件 TIFS 寄存器
	 *
	 * 这是 CONFIG_BT_CTLR_TIFS_HW 模式的核心:
	 *   radio_tmr_tifs_set(EVENT_IFS_US);
	 *   → NRF_RADIO->TIFS = tifs;
	 *
	 * 配合 SHORTS (DISABLED→TXEN 或 DISABLED→RXEN) 使用时,
	 * 硬件自动保证从上一包最后一 bit → 下一包第一 bit = TIFS μs。
	 * 内部自动补偿 ramp-up 延迟。
	 */
	NRF_RADIO->TIFS = T_IFS_US;

	printk("Radio configured: BLE 1M, TIFS=%d μs (HW mode)\n", T_IFS_US);
}

/*===========================================================================
 * 验证 SCAN_REQ
 *===========================================================================*/
static bool validate_scan_req(const struct pdu_adv *rx_pdu)
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

/*===========================================================================
 * 确保 Radio 回到 DISABLED 状态 (安全函数)
 *===========================================================================*/
static void radio_ensure_disabled(void)
{
	NRF_RADIO->SHORTS = 0;

	/* 如果 Radio 已经 DISABLED, TASKS_DISABLE 不会产生新的
	 * EVENTS_DISABLED, 所以先检查状态 */
	if (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled) {
		NRF_RADIO->EVENTS_DISABLED = 0;
		NRF_RADIO->TASKS_DISABLE = 1;
		while (NRF_RADIO->EVENTS_DISABLED == 0) {
		}
	}
}

/*===========================================================================
 * 在指定信道上执行全硬件 TIFS 的 ADV_IND + SCAN_REQ + SCAN_RSP
 *
 * ★ 核心差异 vs phy_adv_scan:
 *   TX→RX 和 RX→TX 都使用 NRF_RADIO->TIFS + SHORTS 实现。
 *
 * 硬件 TIFS 机制:
 *
 *   设置 NRF_RADIO->TIFS = 150 后:
 *
 *   SHORTS: END→DISABLE + DISABLED→RXEN
 *   ───────────────────────────────────────────
 *   TX END (t=0)
 *     → DISABLE (SHORTS)
 *     → DISABLED (~0.5μs)
 *     → 硬件内部延时, 在 t ≈ TIFS-40μs 时触发 RXEN
 *     → READY (~40μs ramp-up)
 *     → START (t ≈ TIFS): 第一个 RX bit 精确对齐
 *
 *   SHORTS: END→DISABLE + DISABLED→TXEN
 *   ───────────────────────────────────────────
 *   RX END (t=0)
 *     → DISABLE (SHORTS)
 *     → DISABLED (~0.5μs)
 *     → 硬件内部延时, 在 t ≈ TIFS-40μs 时触发 TXEN
 *     → READY (~40μs ramp-up)
 *     → START (t ≈ TIFS): 第一个 TX bit 精确对齐
 *
 *   关键优势: 硬件自动补偿 ramp-up 延迟, 不需要软件计算!
 *
 * 对比真实 Controller:
 *
 *   radio_switch_complete_and_rx() [TX→RX]:
 *     NRF_RADIO->SHORTS = READY_START | END_DISABLE | DISABLED_RXEN;
 *
 *   radio_switch_complete_and_tx() [RX→TX]:
 *     NRF_RADIO->SHORTS = READY_START | END_DISABLE | DISABLED_TXEN;
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
	 * 第 1 阶段: TX ADV_IND + 硬件 TIFS 自动切到 RX
	 *
	 * 对应真实 Controller 中:
	 *   radio_tmr_tifs_set(EVENT_IFS_US);
	 *   → NRF_RADIO->TIFS = 150;   (已在 radio_configure 中设置)
	 *
	 *   radio_switch_complete_and_rx(phy_p);
	 *   → NRF_RADIO->SHORTS = READY_START | END_DISABLE | DISABLED_RXEN;
	 *
	 * SHORTS 链: TXEN → READY → START → [TX ADV_IND] → END
	 *            → DISABLE → DISABLED → [HW TIFS 延时]
	 *            → RXEN → READY → START → [RX]
	 *==============================================================*/

	radio_pkt_tx_set(&pdu_adv_ind);

	radio_status_reset();
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_ADDRESS = 0;

	/* ★ HW TIFS TX→RX: 与 radio_switch_complete_and_rx() 完全相同 */
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk |
			    RADIO_SHORTS_DISABLED_RXEN_Msk;

	radio_tx_enable();

	/* 等待 TX END */
	while (NRF_RADIO->EVENTS_END == 0) {
	}
	NRF_RADIO->EVENTS_END = 0;

	/* TX END 后, SHORTS 链正在执行: DISABLE → [HW TIFS] → RXEN
	 * 在此间隙 (~150μs) 中切换 PACKETPTR 到 RX 缓冲区 */
	radio_pkt_rx_set(&pdu_rx_buf);

	/* 等待 TX DISABLED (确认 DISABLE 完成, RXEN 即将触发) */
	while (NRF_RADIO->EVENTS_DISABLED == 0) {
	}
	NRF_RADIO->EVENTS_DISABLED = 0;

	/*==============================================================
	 * 第 2 阶段: RX 等待 SCAN_REQ
	 *
	 * 此时 Radio 正在执行 HW TIFS 延时后的 RX ramp-up。
	 * 先不设置 DISABLED→TXEN, 等检测到 ADDRESS 事件后再设置,
	 * 这样超时时不会误触发 TX。
	 *
	 * SHORTS 暂时: END→DISABLE (无自动 TX 切换)
	 *==============================================================*/

	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;

	/* 等待 ADDRESS 事件 (Access Address 匹配, 说明有扫描器在发包) */
	uint32_t timeout = RX_TIMEOUT_US * 16;
	while (NRF_RADIO->EVENTS_ADDRESS == 0 && timeout > 0) {
		timeout--;
	}

	if (NRF_RADIO->EVENTS_ADDRESS == 0) {
		/* 超时: 无 SCAN_REQ, 安全回到 DISABLED */
		radio_ensure_disabled();
		return;
	}

	/*==============================================================
	 * ADDRESS 匹配! 设置 HW TIFS RX→TX 自动切换
	 *
	 * 对应真实 Controller 中:
	 *   radio_switch_complete_and_tx(phy_rx, flags_rx, phy_tx, flags_tx);
	 *   → NRF_RADIO->SHORTS = READY_START | END_DISABLE | DISABLED_TXEN;
	 *
	 * 此处在 RX 阶段中途更新 SHORTS:
	 *   当 RX END 触发时, SHORTS 链将自动执行:
	 *   RX END → DISABLE → DISABLED → [HW TIFS] → TXEN → TX SCAN_RSP
	 *==============================================================*/

	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk |
			    RADIO_SHORTS_DISABLED_TXEN_Msk;

	/* 等待 RX END (SCAN_REQ 接收完毕) */
	while (NRF_RADIO->EVENTS_END == 0) {
	}
	NRF_RADIO->EVENTS_END = 0;

	/* RX END 后, SHORTS 链: DISABLE → DISABLED → [HW TIFS] → TXEN
	 *
	 * 从 RX END 到 TX START ≈ 150μs (由 HW TIFS 保证)
	 * 在此时间窗口内 (~150μs) 完成以下操作:
	 *   1. 验证 SCAN_REQ (CRC + 地址匹配)
	 *   2. 切换 PACKETPTR 到 SCAN_RSP
	 *
	 * 如果验证失败, 在 TXEN 触发前 (~110μs) 取消 TX */

	bool valid = radio_crc_is_valid() && validate_scan_req(&pdu_rx_buf);

	if (!valid) {
		/* 验证失败: 取消 HW TIFS TX 链
		 *
		 * 此时距 RX END 仅 ~5μs, 距 HW 触发 TXEN 还有 ~105μs。
		 * 清除 SHORTS 并确保 Radio 回到 DISABLED 状态。
		 *
		 * 时间线:
		 *   RX END (t=0) → 验证 (~5μs) → 清除 SHORTS (~6μs)
		 *                                  ↑ 我们在这里取消
		 *   如果不取消: TXEN 会在 t≈110μs 触发
		 */
		radio_ensure_disabled();
		return;
	}

	scan_req_count++;

	/*==============================================================
	 * 第 3 阶段: TX SCAN_RSP (全靠 HW TIFS 自动执行)
	 *
	 * SHORTS 链已在阶段 2 设置 DISABLED→TXEN:
	 *   RX END → DISABLE → DISABLED → [HW TIFS 150μs] → TXEN
	 *   → READY → START → [TX SCAN_RSP] → END → DISABLE
	 *
	 * 我们只需:
	 *   1. 切换 PACKETPTR 到 SCAN_RSP
	 *   2. 等待 TX 完成
	 *
	 * 对比 phy_adv_scan 中使用 k_busy_wait:
	 *   k_busy_wait(T_IFS_US - TX_RAMPUP_US - 15);  ← 软件近似
	 *   radio_tx_enable();                           ← 手动触发
	 *
	 * 本 Demo:
	 *   hardware TIFS 自动触发 TXEN                   ← 硬件精确
	 *==============================================================*/

	radio_pkt_tx_set(&pdu_scan_rsp);

	/* 等待 RX DISABLED (SHORTS 链: DISABLE 完成) */
	while (NRF_RADIO->EVENTS_DISABLED == 0) {
	}
	NRF_RADIO->EVENTS_DISABLED = 0;

	/* 更新 SHORTS: TX 完成后仅 DISABLE, 不再自动切换 */
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;

	/* 等待 TX SCAN_RSP 完成 (END → DISABLE → DISABLED) */
	while (NRF_RADIO->EVENTS_END == 0) {
	}
	NRF_RADIO->EVENTS_END = 0;

	while (NRF_RADIO->EVENTS_DISABLED == 0) {
	}
	NRF_RADIO->SHORTS = 0;

	scan_rsp_count++;
}

/*===========================================================================
 * 广播工作处理函数
 *===========================================================================*/
static void adv_worker(struct k_work *work)
{
	ARG_UNUSED(work);

	adv_on_channel(37);
	adv_on_channel(38);
	adv_on_channel(39);

	adv_event_count++;

	if (adv_event_count <= 3 || adv_event_count % 10 == 0) {
		printk("[HW-TIFS] events: %u | SCAN_REQ rx: %u | SCAN_RSP tx: %u\n",
		       adv_event_count, scan_req_count, scan_rsp_count);
	}
}

/*===========================================================================
 * 定时器回调
 *===========================================================================*/
static void adv_timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_work_submit(&adv_work);
}

/*===========================================================================
 * 主函数
 *===========================================================================*/
int main(void)
{
	printk("\n=== BLE PHY HW TIFS Demo ===\n");
	printk("全部 TX↔RX 切换使用 NRF_RADIO->TIFS 硬件定时\n\n");

	hfclk_start();
	radio_configure();
	build_adv_ind_pdu();
	build_scan_rsp_pdu();

	k_work_init(&adv_work, adv_worker);
	k_timer_init(&adv_timer, adv_timer_expiry, NULL);

	k_timer_start(&adv_timer, K_NO_WAIT, K_MSEC(ADV_INTERVAL_MS));

	printk("Advertising started (HW TIFS mode)\n\n");

	while (1) {
		k_sleep(K_SECONDS(5));
		printk("[Main] alive — adv: %u, scan_req: %u, scan_rsp: %u\n",
		       adv_event_count, scan_req_count, scan_rsp_count);
	}

	return 0;
}
