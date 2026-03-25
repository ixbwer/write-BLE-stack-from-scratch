/*
 * BLE PHY ADV_IND + IFS + SCAN_RSP Demo
 *
 * 基于 phy_scheduled_tx, 本 Demo 增加了以下机制:
 *
 *   1. ADV_IND: 可扫描可连接广播 (替代 NONCONN_IND)
 *   2. IFS (Inter-Frame Spacing): BLE 规范 T_IFS = 150 μs
 *      - 使用 NRF_RADIO->TIFS 寄存器实现硬件精确 IFS
 *      - 使用 SHORTS: END→DISABLE→RXEN 自动从 TX 切换到 RX
 *   3. SCAN_RSP: 接收 SCAN_REQ 后自动回复 SCAN_RSP
 *
 * 完整的单信道工作流程:
 *
 *   ┌─────────┐  T_IFS=150μs  ┌──────────┐  T_IFS=150μs  ┌──────────┐
 *   │ ADV_IND ├───────────────►│ SCAN_REQ ├───────────────►│ SCAN_RSP │
 *   │  (TX)   │               │   (RX)   │               │   (TX)   │
 *   └─────────┘               └──────────┘               └──────────┘
 *     广播器发送                 扫描器发送                  广播器回复
 *
 *   如果 T_IFS 超时没有收到 SCAN_REQ, 直接跳到下一信道。
 *
 * 广播事件 = 在 ch37/38/39 上各执行一次上述流程
 *
 * 对比真实 Controller 的架构对应:
 *
 *   ┌─────────────────────────────────────────────────────┐
 *   │  真实 Controller 架构    │   本 Demo 对应实现        │
 *   │─────────────────────────│───────────────────────────│
 *   │  Ticker (RTC0 ISR)      │   k_timer (内核定时器)     │
 *   │  Mayfly (延迟回调)       │   k_work  (工作队列)      │
 *   │  LLL Event (射频操作)   │   adv_worker() 函数       │
 *   │  NRF_RADIO->TIFS        │   NRF_RADIO->TIFS (同)    │
 *   │  SHORTS TX→RX→TX        │   SHORTS + 轮询           │
 *   └─────────────────────────────────────────────────────┘
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
#include "hal/ccm.h"       /* struct ccm (需在 radio.h 之前) */
#include "hal/radio.h"     /* radio_phy_set, radio_aa_set, ... */

/* ---- Controller PDU 结构体定义 ---- */
#include "pdu_df.h"        /* struct pdu_cte_info */
#include "pdu_vendor.h"    /* Nordic 特定的 PDU 扩展 */
#include "pdu.h"           /* struct pdu_adv, PDU_AC_ACCESS_ADDR, ... */

/*===========================================================================
 * 配置参数
 *===========================================================================*/

/* 广播间隔 (ms) */
#define ADV_INTERVAL_MS  100

/* T_IFS: BLE 规范定义的帧间间隔 = 150 μs
 * 见 Core Spec Vol 6, Part B, Section 4.1 */
#define T_IFS_US         150

/* RX 等待超时 (μs): T_IFS + 最大包时长 + 余量
 * SCAN_REQ 为 12 字节 payload, BLE 1M PHY:
 *   前导码(1B) + AA(4B) + Header(2B) + Payload(12B) + CRC(3B) = 22B = 176 μs
 * 总超时 = T_IFS(150) + 包时长(176) + 余量(50) ≈ 376 μs
 * 再加上地址匹配需要的时间, 给 500 μs */
#define RX_TIMEOUT_US    500

/* TX ramp-up 时间估计 (μs): nRF52 TXEN → READY 约 40 μs */
#define TX_RAMPUP_US     40

/* 是否启用 SCAN_RSP 回复功能
 * 1 = ADV_IND 后监听 SCAN_REQ 并回复 SCAN_RSP (完整 IFS 流程)
 * 0 = ADV_IND 发完直接跳下一信道 (类似 NONCONN_IND 行为, 但 PDU 类型仍是 ADV_IND) */
#define ENABLE_SCAN_RSP  1

/* 广播设备名称 */
#define ADV_NAME         "ZephyrIFS"

/* 随机静态地址 (MSB → LSB 表示: 66:55:44:33:22:11)
 * BLE PDU 中以小端存储 (LSB first) */
static const uint8_t adv_addr[BDADDR_SIZE] = {
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66  /* PDU 中小端存储 */
};

/*===========================================================================
 * 广播 AD 数据 (ADV_IND 中携带的 AD 结构)
 *===========================================================================*/
static const uint8_t adv_data[] = {
	/* Flags: LE General Discoverable + BR/EDR Not Supported */
	0x02, 0x01, 0x06,
	/* Complete Local Name: "ZephyrIFS" */
	0x0A, 0x09, 'Z', 'e', 'p', 'h', 'y', 'r', 'I', 'F', 'S'
};

#define ADV_DATA_LEN sizeof(adv_data)

/*===========================================================================
 * 扫描响应数据 (SCAN_RSP 中携带的 AD 结构)
 *===========================================================================*/
static const uint8_t scan_rsp_data[] = {
	/* Complete Local Name: "ZephyrIFS-ScanRsp" (18 chars) */
	0x12, 0x09, 'Z', 'e', 'p', 'h', 'y', 'r', 'I', 'F', 'S',
	             '-', 'S', 'c', 'a', 'n', 'R', 's', 'p',
};

#define SCAN_RSP_DATA_LEN sizeof(scan_rsp_data)

/*===========================================================================
 * PDU 缓冲区
 *===========================================================================*/

/* ADV_IND PDU (TX) */
static struct pdu_adv pdu_adv_ind __aligned(4);

/* SCAN_RSP PDU (TX) */
static struct pdu_adv pdu_scan_rsp __aligned(4);

/* RX 缓冲区: 接收 SCAN_REQ */
static struct pdu_adv pdu_rx_buf __aligned(4);

/*===========================================================================
 * 调度器组件
 *===========================================================================*/
static struct k_timer adv_timer;      /* 广播定时器 (模拟 Ticker) */
static struct k_work  adv_work;       /* 广播工作项 (模拟 Mayfly) */

/* 统计计数器 */
static volatile uint32_t adv_event_count;
static volatile uint32_t timer_fire_count;
static volatile uint32_t scan_req_count;
static volatile uint32_t scan_rsp_count;

/*===========================================================================
 * 构造 ADV_IND PDU
 *
 * ADV_IND 格式 (Core Spec Vol 6, Part B, 2.3.1.1):
 *   Header: Type=0x00 (ADV_IND), TxAdd, RxAdd, Length
 *   Payload: AdvA (6 bytes) + AdvData (0~31 bytes)
 *
 * ADV_IND 与 NONCONN_IND 的关键区别:
 *   - ADV_IND 可接收 SCAN_REQ 和 CONNECT_IND
 *   - NONCONN_IND 发完就走, 不听不回
 *===========================================================================*/
static void build_adv_ind_pdu(void)
{
	memset(&pdu_adv_ind, 0, sizeof(pdu_adv_ind));

	pdu_adv_ind.type    = PDU_ADV_TYPE_ADV_IND;  /* 0x00: 可扫描可连接 */
	pdu_adv_ind.tx_addr = 1;  /* TxAdd=1: Random Static Address */
	pdu_adv_ind.rx_addr = 0;
	pdu_adv_ind.len     = BDADDR_SIZE + ADV_DATA_LEN;

	memcpy(pdu_adv_ind.adv_ind.addr, adv_addr, BDADDR_SIZE);
	memcpy(pdu_adv_ind.adv_ind.data, adv_data, ADV_DATA_LEN);

	printk("ADV_IND PDU built: type=0x%02X len=%d\n",
	       pdu_adv_ind.type, pdu_adv_ind.len);
}

/*===========================================================================
 * 构造 SCAN_RSP PDU
 *
 * SCAN_RSP 格式 (Core Spec Vol 6, Part B, 2.3.2.2):
 *   Header: Type=0x04 (SCAN_RSP), TxAdd, Length
 *   Payload: AdvA (6 bytes) + ScanRspData (0~31 bytes)
 *===========================================================================*/
static void build_scan_rsp_pdu(void)
{
	memset(&pdu_scan_rsp, 0, sizeof(pdu_scan_rsp));

	pdu_scan_rsp.type    = PDU_ADV_TYPE_SCAN_RSP;  /* 0x04 */
	pdu_scan_rsp.tx_addr = 1;  /* TxAdd=1: Random Static Address (与 ADV_IND 一致) */
	pdu_scan_rsp.rx_addr = 0;
	pdu_scan_rsp.len     = BDADDR_SIZE + SCAN_RSP_DATA_LEN;

	memcpy(pdu_scan_rsp.scan_rsp.addr, adv_addr, BDADDR_SIZE);
	memcpy(pdu_scan_rsp.scan_rsp.data, scan_rsp_data, SCAN_RSP_DATA_LEN);

	printk("SCAN_RSP PDU built: type=0x%02X len=%d\n",
	       pdu_scan_rsp.type, pdu_scan_rsp.len);
}

/*===========================================================================
 * 启动 HFCLK (16MHz 外部晶振) — RADIO 的前置条件
 *===========================================================================*/
static void hfclk_start(void)
{
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;

	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
		/* busy wait */
	}

	printk("HFCLK (16MHz HFXO) started\n");
}

/*===========================================================================
 * 使用 HAL 接口配置 RADIO
 *===========================================================================*/
static void radio_configure(void)
{
	radio_phy_set(0, 0);   /* BLE 1M PHY */
	radio_tx_power_set(0); /* 0 dBm */

	radio_pkt_configure(RADIO_PKT_CONF_LENGTH_8BIT,
			    PDU_AC_LEG_PAYLOAD_SIZE_MAX,
			    RADIO_PKT_CONF_FLAGS(
				    RADIO_PKT_CONF_PDU_TYPE_AC,
				    RADIO_PKT_CONF_PHY_LEGACY,
				    RADIO_PKT_CONF_CTE_DISABLED));

	uint32_t aa = sys_cpu_to_le32(PDU_AC_ACCESS_ADDR);
	radio_aa_set((const uint8_t *)&aa);

	radio_crc_configure(PDU_CRC_POLYNOMIAL, PDU_AC_CRC_IV);

	/* 设置硬件 T_IFS
	 *
	 * NRF_RADIO->TIFS 寄存器:
	 *   当配合 SHORTS (DISABLED→TXEN 或 DISABLED→RXEN) 使用时,
	 *   硬件会在 DISABLED 事件后自动等待 TIFS 微秒再触发下一个操作。
	 *
	 * 真实 Controller 中的等价代码:
	 *   radio_tmr_tifs_set(EVENT_IFS_US);
	 *   // 在 CONFIG_BT_CTLR_TIFS_HW 模式下, 最终也是:
	 *   // NRF_RADIO->TIFS = tifs;
	 */
	NRF_RADIO->TIFS = T_IFS_US;

	printk("Radio configured: BLE 1M, AA=0x%08X, TIFS=%d μs\n",
	       PDU_AC_ACCESS_ADDR, T_IFS_US);
}

/*===========================================================================
 * 验证 SCAN_REQ PDU 是否合法
 *
 * SCAN_REQ 格式 (Core Spec Vol 6, Part B, 2.3.2.1):
 *   Header: Type=0x03 (SCAN_REQ), TxAdd, RxAdd, Len=12
 *   Payload: ScanA (6 bytes) + AdvA (6 bytes)
 *
 * 验证规则:
 *   1. PDU type 必须是 SCAN_REQ (0x03)
 *   2. Payload 长度必须是 12 字节
 *   3. AdvA 必须匹配我们的广播地址
 *   4. CRC 必须正确
 *===========================================================================*/
static bool validate_scan_req(const struct pdu_adv *rx_pdu)
{
	/* 检查 PDU 类型 */
	if (rx_pdu->type != PDU_ADV_TYPE_SCAN_REQ) {
		return false;
	}

	/* 检查长度: ScanA(6) + AdvA(6) = 12 */
	if (rx_pdu->len != sizeof(struct pdu_adv_scan_req)) {
		return false;
	}

	/* 检查 AdvA 是否匹配我们的地址 */
	if (memcmp(rx_pdu->scan_req.adv_addr, adv_addr, BDADDR_SIZE) != 0) {
		return false;
	}

	return true;
}

/*===========================================================================
 * 确保 Radio 处于 DISABLED 状态
 *
 * 关键安全函数: nRF52 的 RADIO 只有在 DISABLED 状态下才能响应
 * TXEN/RXEN 任务。如果在 RX/TX 中途返回, 下一次操作会因状态
 * 错误而永久卡死。此函数保证无论 Radio 处于何种状态, 都能
 * 安全回到 DISABLED。
 *===========================================================================*/
static void radio_force_disable(void)
{
	NRF_RADIO->SHORTS = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_DISABLE = 1;
	while (NRF_RADIO->EVENTS_DISABLED == 0) {
		/* busy wait */
	}
}

/*===========================================================================
 * 在指定广播信道上执行完整的 ADV_IND + IFS + SCAN_RSP 流程
 *
 * 流程:
 *   1. TX ADV_IND
 *   2. 硬件自动等待 T_IFS (150 μs) 后切换到 RX (通过 SHORTS + TIFS 寄存器)
 *   3. 等待 SCAN_REQ (带超时)
 *   4. 如果收到合法 SCAN_REQ:
 *      a. 等待 T_IFS (用 k_busy_wait 近似)
 *      b. TX SCAN_RSP
 *   5. 如果超时或 CRC 错误, 直接跳到下一信道
 *
 * IFS 时序图:
 *
 *   ┌──────────┐  T_IFS=150μs  ┌───────────┐  T_IFS≈150μs  ┌───────────┐
 *   │ ADV_IND  ├──────────────►│ SCAN_REQ  ├──────────────►│ SCAN_RSP  │
 *   │  (TX)    │  HW TIFS      │   (RX)    │  SW busy_wait │   (TX)    │
 *   └──────────┘               └───────────┘               └───────────┘
 *
 * 硬件 TIFS (第 1→2 阶段):
 *   SHORTS: END→DISABLE + DISABLED→RXEN + NRF_RADIO->TIFS=150
 *   硬件确保 ADV_IND 最后一个 bit → RX 第一个 bit 正好 150 μs
 *
 * 软件 TIFS (第 3 阶段):
 *   使用 k_busy_wait 近似 T_IFS 延时后手动触发 TX
 *   (真实 Controller 使用 PPI + Timer 实现精确时序)
 *===========================================================================*/
static void adv_on_channel(uint32_t channel)
{
	uint32_t freq;

	switch (channel) {
	case 37: freq = 2;  break;  /* 2402 MHz */
	case 38: freq = 26; break;  /* 2426 MHz */
	case 39: freq = 80; break;  /* 2480 MHz */
	default: return;
	}

	radio_freq_chan_set(freq);
	radio_whiten_iv_set(channel);

	/* 禁用 RADIO 中断 (全程轮询模式) */
	NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;

	/*==============================================================
	 * 第 1 阶段: TX ADV_IND, 硬件 TIFS 自动切到 RX
	 *
	 * SHORTS 链: READY→START | END→DISABLE | DISABLED→RXEN
	 * 配合 NRF_RADIO->TIFS = 150, 硬件保证:
	 *   TXEN → READY → START → [发送 ADV_IND] → END → DISABLE
	 *   → DISABLED (event) → 等到距 last-bit-on-air 150μs 时
	 *   → RXEN → READY → START → [接收]
	 *
	 * 关键: EVENTS_DISABLED 在 TX DISABLE 时就会置 1,
	 *       必须在进入 RX 等待前清除, 否则会误判 RX 完成!
	 *==============================================================*/

	radio_pkt_tx_set(&pdu_adv_ind);

	/* 清除所有事件标志 */
	radio_status_reset();
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_ADDRESS = 0;

#if ENABLE_SCAN_RSP
	/* SHORTS: TX 完成后通过硬件 TIFS 自动切到 RX */
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk |
			    RADIO_SHORTS_DISABLED_RXEN_Msk;
#else
	/* SCAN_RSP 关闭: TX 完成后直接 DISABLE */
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;
#endif

	radio_tx_enable();

	/* 等待 TX END (ADV_IND 发送完毕) */
	while (NRF_RADIO->EVENTS_END == 0) {
		/* busy wait */
	}
	NRF_RADIO->EVENTS_END = 0;

#if !ENABLE_SCAN_RSP
	/* SCAN_RSP 关闭: 等待 DISABLE 完成后直接返回 */
	while (NRF_RADIO->EVENTS_DISABLED == 0) {
		/* busy wait */
	}
	NRF_RADIO->SHORTS = 0;
	return;
#endif

	/* TX END 后, 硬件 SHORTS 链正在执行:
	 *   END → DISABLE → DISABLED(event) → 等 TIFS → RXEN → ...
	 *
	 * 在 TIFS 间隙 (~190 μs) 内切换 PACKETPTR 到 RX 缓冲区。
	 * PACKETPTR 在 RX START 时才被 DMA 读取, 来得及。 */
	radio_pkt_rx_set(&pdu_rx_buf);

	/* 等待 TX 阶段的 DISABLED 事件 (END 后几个时钟周期就会触发) */
	while (NRF_RADIO->EVENTS_DISABLED == 0) {
		/* busy wait */
	}
	/* ★ 关键: 清除 TX 阶段产生的 DISABLED 事件!
	 * 如果不清除, 后面的 RX 等待循环会立即误判为 "RX 完成",
	 * 导致 Radio 停留在 RX 状态, 下一个信道调用 TXEN 时
	 * Radio 不在 DISABLED 状态 → EVENTS_END 永远不来 → 永久卡死 */
	NRF_RADIO->EVENTS_DISABLED = 0;

	/* 更新 SHORTS: RX 收完包后 DISABLE, 不再自动切到其他模式 */
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;

	/*==============================================================
	 * 第 2 阶段: RX 等待 SCAN_REQ (带超时)
	 *
	 * 此时 Radio 正在 TIFS 延迟后进入 RX 模式, 等待接收。
	 * SHORTS: END→DISABLE, 收到完整包后自动 DISABLE, EVENTS_DISABLED 置 1。
	 * 如果超时没收到包 (无 AA 匹配), 手动 DISABLE。
	 *==============================================================*/

	/* 超时循环  (nRF52 @ 64MHz, 每次迭代约 0.1~0.2 μs, *16 给足余量) */
	uint32_t timeout_loops = RX_TIMEOUT_US * 16;
	bool rx_done = false;

	while (timeout_loops > 0) {
		if (NRF_RADIO->EVENTS_DISABLED) {
			rx_done = true;
			break;
		}
		timeout_loops--;
	}

	if (!rx_done) {
		/* 超时: Radio 还在 RX, 手动 DISABLE 回到安全状态 */
		radio_force_disable();
		return;
	}

	/* Radio 已 DISABLED (RX END→DISABLE SHORTS), 状态安全 */
	NRF_RADIO->SHORTS = 0;

	/* 检查 CRC 和 SCAN_REQ 内容 */
	if (!radio_crc_is_valid()) {
		return;  /* Radio 已 DISABLED, 可安全返回 */
	}

	if (!validate_scan_req(&pdu_rx_buf)) {
		return;  /* Radio 已 DISABLED, 可安全返回 */
	}

	scan_req_count++;

	/*==============================================================
	 * 第 3 阶段: TX SCAN_RSP (软件 T_IFS)
	 *
	 * 收到合法 SCAN_REQ 后, 需要在 T_IFS(150μs) 内发出 SCAN_RSP。
	 *
	 * 此处使用 k_busy_wait 近似 T_IFS 延时:
	 *   T_IFS = 150 μs (从 SCAN_REQ 最后一 bit → SCAN_RSP 第一 bit)
	 *   TX ramp-up ≈ 40 μs (TXEN → READY → START → 第一 bit)
	 *   从 SCAN_REQ END 到此处 ≈ 3~5 μs
	 *   需等待: 150 - 40 - 5 ≈ 105 μs
	 *
	 * 真实 Controller 中使用 PPI + Timer 实现 μs 级精度:
	 *   radio_tmr_tifs_set(EVENT_IFS_US);
	 *   radio_switch_complete_and_tx(phy_p, 0, phy_p, phy_flags);
	 *==============================================================*/

	radio_pkt_tx_set(&pdu_scan_rsp);
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->EVENTS_END = 0;

	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;

	/* 等待近似 T_IFS 时间再触发 TX */
	k_busy_wait(T_IFS_US - TX_RAMPUP_US - 15);

	radio_tx_enable();

	/* 等待 SCAN_RSP 发送完成 */
	while (NRF_RADIO->EVENTS_DISABLED == 0) {
		/* busy wait */
	}

	NRF_RADIO->SHORTS = 0;
	scan_rsp_count++;
}

/*===========================================================================
 * 广播工作处理函数 — 在系统工作队列线程中执行
 *
 * 一个广播事件 = 在 ch37/38/39 上各执行一次 ADV_IND + IFS 流程
 *===========================================================================*/
static void adv_worker(struct k_work *work)
{
	ARG_UNUSED(work);

	/* 执行完整的广播事件: 三信道依次执行 ADV_IND + IFS + SCAN_RSP */
	adv_on_channel(37);
	adv_on_channel(38);
	adv_on_channel(39);

	adv_event_count++;

	/* 前 3 次事件每次打印, 之后每 10 次打印一次 */
	if (adv_event_count <= 3 || adv_event_count % 10 == 0) {
		printk("[ADV] events: %u | SCAN_REQ rx: %u | SCAN_RSP tx: %u\n",
		       adv_event_count, scan_req_count, scan_rsp_count);
	}
}

/*===========================================================================
 * 定时器到期回调 — 在 ISR 上下文中执行 (模拟 Ticker)
 *===========================================================================*/
static void adv_timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	timer_fire_count++;
	k_work_submit(&adv_work);
}

/*===========================================================================
 * 主函数
 *===========================================================================*/
int main(void)
{
	printk("=== BLE PHY ADV_IND + IFS + SCAN_RSP Demo ===\n");
	printk("Architecture: k_timer(Ticker) → k_work(Mayfly) → ADV+IFS(LLL)\n");
	printk("Device Name:  %s\n", ADV_NAME);
	printk("Address:      D7:B6:E5:C4:A3:F2 (Random Static)\n");
	printk("Interval:     %d ms\n", ADV_INTERVAL_MS);
	printk("T_IFS:        %d μs (hardware)\n\n", T_IFS_US);

	/* ---- 第 1 步: 硬件初始化 ---- */
	hfclk_start();
	radio_configure();
	build_adv_ind_pdu();
	build_scan_rsp_pdu();

	/* ---- 第 2 步: 初始化调度器组件 ---- */
	k_work_init(&adv_work, adv_worker);
	k_timer_init(&adv_timer, adv_timer_expiry, NULL);

	printk("Scheduler initialized\n");

	/* ---- 第 3 步: 启动调度器 ---- */
	k_timer_start(&adv_timer, K_NO_WAIT, K_MSEC(ADV_INTERVAL_MS));

	printk("Advertising started (ADV_IND + IFS + SCAN_RSP)\n");
	printk("Use nRF Connect / hcitool to send SCAN_REQ and see SCAN_RSP!\n\n");

	/* ---- 主线程: 空闲循环 ---- */
	while (1) {
		k_sleep(K_SECONDS(5));
		printk("[Main] alive — adv: %u, scan_req: %u, scan_rsp: %u\n",
		       adv_event_count, scan_req_count, scan_rsp_count);
	}

	return 0;
}
