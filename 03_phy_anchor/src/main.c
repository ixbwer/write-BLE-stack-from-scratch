/*
 * BLE PHY Anchor Point (锚点) 机制 Demo
 *
 * 本 Demo 专注于演示 BLE 连接中最核心的定时概念 —— Anchor Point (锚点)。
 *
 * ┌ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┐
 * │ 什么是 Anchor Point?                                                  │
 * │                                                                       │
 * │ Anchor Point 是 BLE 连接中每个连接事件的时间基准点。                  │
 * │ Core Spec Vol 6, Part B, Section 4.5.1:                               │
 * │   "The anchor point of the first connection event shall be            │
 * │    transmitWindowDelay + transmitWindowOffset + n × connInterval      │
 * │    after the end of the CONNECT_IND PDU"                              │
 * │                                                                       │
 * │ 锚点的核心作用:                                                       │
 * │   1. 首次锚点: 从 CONNECT_IND 接收时刻精确计算                       │
 * │   2. 后续锚点: 每隔 connInterval 递推 (绝对时间, 非相对延时)         │
 * │   3. 锚点更新: 成功收到数据包后, 用实际接收时刻校准锚点              │
 * │   4. Window Widening: 连续丢包时, RX 窗口逐渐加宽以适应时钟漂移      │
 * └ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┘
 *
 * 锚点时间线:
 *
 *   CONNECT_IND           1st Anchor        2nd Anchor        3rd Anchor
 *   end time (T0)            (A0)              (A1)              (A2)
 *       │                     │                 │                 │
 *   ────┼─────────────────────┼─────────────────┼─────────────────┼────►
 *       │                     │                 │                 │  time
 *       │← winOffset+winDelay→│←── interval ───→│←── interval ───→│
 *
 *   A0 = T0 + winOffset×1.25ms + transmitWindowDelay(1.25ms)
 *   A1 = A0 + connInterval
 *   A2 = A1 + connInterval
 *   ...
 *   An = A0 + n × connInterval
 *
 * 锚点更新 (Anchor Point Update):
 *
 *   当 Slave 成功收到 Master 的数据包时:
 *     - 用该包的实际接收时刻作为新锚点
 *     - 重置 Window Widening 到 0
 *     - 这确保了时钟漂移不会累积
 *
 *   当 Slave 连续丢包时:
 *     - 继续使用上一个成功锚点 + N×interval 推算
 *     - Window Widening 每次增加 (SCA_master + SCA_slave) × interval
 *     - RX 窗口逐渐变宽, 容忍更大的时钟偏差
 *
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │               Window Widening 效果示意                       │
 *   │                                                              │
 *   │   成功 RX:  ├─ 窄 ─┤    (ww=0, 精确对齐)                    │
 *   │   丢 1 包:  ├── 宽 ──┤   (ww += periodic)                   │
 *   │   丢 2 包:  ├─── 更宽 ───┤  (ww += periodic)                │
 *   │   ...                                                        │
 *   │   成功 RX:  ├─ 窄 ─┤    (ww 重置为 0, 锚点更新)             │
 *   └──────────────────────────────────────────────────────────────┘
 *
 * Zephyr 控制器中的对应实现:
 *
 *   ┌──────────────────────────┬───────────────────────────────────────┐
 *   │  概念                    │  Zephyr 源码位置                      │
 *   │──────────────────────────│───────────────────────────────────────│
 *   │  首次锚点计算            │  ull_periph.c: ull_periph_setup()     │
 *   │  锚点 RTC tick 表示     │  ftr->ticks_anchor = tmr_start_get() │
 *   │  锚点到 radio 启动      │  radio_tmr_start(0, ticks, remainder) │
 *   │  connInterval 递推      │  ticker 周期参数 (ticks + remainder)   │
 *   │  Window Widening 累积   │  lll->periph.window_widening_event_us │
 *   │  锚点更新 (成功RX后)    │  isr_done: ww=0, win_size=0           │
 *   │  HCTO (RX超时保护)      │  radio_tmr_hcto_configure(hcto)       │
 *   └──────────────────────────┴───────────────────────────────────────┘
 *
 * 本 Demo 增加了详细的锚点相关日志, 让你直观看到:
 *   - 首次锚点如何从 CONNECT_IND 计算
 *   - 每次连接事件的锚点 tick 值
 *   - Window Widening 如何随丢包增长、随成功 RX 重置
 *   - 锚点更新的精确时刻
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <soc.h>
#include <hal/nrf_clock.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_ppi.h>

/* ---- Controller HAL 层头文件 ---- */
#include "hal/ccm.h"
#include "hal/radio.h"

/* ---- Controller PDU 结构体定义 ---- */
#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

/* ---- Zephyr 控制器 Ticker 定时层 ---- */
#include "hal/ticker.h"

/*===========================================================================
 * 配置参数
 *===========================================================================*/

/* 广播间隔 (ms) */
#define ADV_INTERVAL_MS  100

/* T_IFS = 150 μs */
#define T_IFS_US         150

/* ADV RX 超时 (μs) */
#define ADV_RX_TIMEOUT_US  600

/* BLE 定时常量 */
#define WIN_DELAY_LEGACY            1250  /* transmitWindowDelay (μs) */
#define CONN_INT_UNIT_US            1250  /* connInterval unit (1.25ms) */
#define EVENT_JITTER_US             16    /* Active clock jitter margin (μs) */
#define EVENT_TICKER_RES_MARGIN_US  31    /* RTC tick resolution ~30.52μs ceil */
#define ADDR_US_1M                  40    /* Preamble(8)+AA(32) for 1M PHY */
#define RTC0_CMP_OFFSET_MIN         3     /* NRF_RTC0 CC 最小比较偏移 */

#define ADV_NAME         "Anchor"

/* TX/RX ramp-up 延迟 (μs): fast ramp-up ≈ 40 μs */
#define RX_RAMP_US       40
#define TX_RAMP_US       40

/* Chain delay: RADIO END 事件相对于实际空中包结束的延迟
 * nRF52832 1M PHY: RX chain delay = 9.4μs (ceil→10), TX chain delay = 0.6μs (ceil→1)
 * 这些延迟影响 SW TIFS CC 值的计算 */
#define RX_CHAIN_DELAY_1M_US  10   /* HAL_RADIO_NRF52832_RX_CHAIN_DELAY_1M_NS=9400 */
#define TX_CHAIN_DELAY_US      1   /* HAL_RADIO_NRF52832_TX_CHAIN_DELAY_NS=600 */

/*===========================================================================
 * PPI 通道分配 (SW TIFS)
 *
 * 使用 TIMER1 + PPI 替代 NRF_RADIO->TIFS 寄存器, 实现软件 TIFS 切换。
 * PPI_CH_TIMER_CLEAR 始终启用, 其他两个按需启停。
 *===========================================================================*/
#define PPI_CH_TIMER_CLEAR  14   /* RADIO END → TIMER1 CLEAR */
#define PPI_CH_RXEN         15   /* TIMER1 CC[0] → RADIO RXEN */
#define PPI_CH_TXEN         16   /* TIMER1 CC[1] → RADIO TXEN */

#define CC_IDX_RXEN  0   /* Timer CC[0]: TX→RX 转换 */
#define CC_IDX_TXEN  1   /* Timer CC[1]: RX→TX 转换 */

static const uint8_t adv_addr[BDADDR_SIZE] = {
	0x11, 0x22, 0x33, 0x44, 0x55, 0xA6
};

/*===========================================================================
 * 广播 AD 数据
 *===========================================================================*/
static const uint8_t adv_data[] = {
	0x02, 0x01, 0x06,
	0x07, 0x09, 'A', 'n', 'c', 'h', 'o', 'r'
};

#define ADV_DATA_LEN sizeof(adv_data)

/*===========================================================================
 * 连接参数结构体
 *===========================================================================*/
struct conn_param {
	uint8_t  access_addr[4];
	uint8_t  crc_init[3];
	uint8_t  win_size;
	uint16_t win_offset;
	uint16_t interval;
	uint16_t latency;
	uint16_t timeout;
	uint8_t  chan_map[PDU_CHANNEL_MAP_SIZE];
	uint8_t  hop;
	uint8_t  sca;
	uint8_t  chan_count;
	uint8_t  peer_addr[BDADDR_SIZE];
};

static struct conn_param conn_params;

/*===========================================================================
 * 连接态变量
 *===========================================================================*/
static uint8_t tx_sn;
static uint8_t rx_nesn;
static uint16_t conn_event_counter;
static uint8_t last_unmapped_chan;
static volatile bool conn_terminated;

static struct pdu_data tx_pdu_buf __aligned(4);
static bool tx_pdu_pending;

/*===========================================================================
 * PDU 缓冲区
 *===========================================================================*/
static struct pdu_adv pdu_adv_ind __aligned(4);
static struct pdu_adv pdu_adv_rx __aligned(4);
static struct pdu_data pdu_data_rx __aligned(4);

/* CONNECT_IND 接收时刻 (NRF_RTC0 Counter) — 这就是 "T0", 锚点的起始基准 */
static uint32_t connect_end_rtc;

/* SCA PPM 映射表 */
static const uint16_t sca_ppm_table[] = {500, 250, 150, 100, 75, 50, 30, 20};
#define LOCAL_SCA_PPM 50

/*===========================================================================
 * 锚点追踪结构体 (本 Demo 新增, 用于详细日志)
 *===========================================================================*/
struct anchor_tracker {
	uint32_t initial_anchor_rtc;     /* 首次计算出的锚点 RTC tick */
	uint32_t initial_anchor_us;      /* 首次锚点的 μs 偏移量 */
	uint32_t current_anchor_rtc;     /* 当前锚点 RTC tick */
	uint32_t last_rx_anchor_rtc;     /* 最后一次成功 RX 时的锚点 */
	uint32_t ww_current_us;          /* 当前 Window Widening (μs) */
	uint32_t ww_periodic_us;         /* 每事件 WW 增量 */
	uint32_t ww_max_us;              /* WW 上限 */
	uint32_t win_size_event_us;      /* 当前 transmit window size */
	uint32_t anchor_update_count;    /* 锚点更新 (成功RX) 次数 */
	uint32_t consecutive_misses;     /* 连续丢包计数 */
	uint32_t max_consecutive_misses; /* 最大连续丢包 */
};

static struct anchor_tracker anchor;

/* 统计 */
static volatile uint32_t adv_event_count;
static volatile uint32_t conn_event_rx_ok;
static volatile uint32_t conn_event_rx_timeout;
static volatile uint32_t conn_event_rx_crc_err;

/* Drift correction: TIMER0 captures from last successful RX
 * Saved inside conn_event() before TX phase (TX READY would overwrite CC) */
static uint32_t last_rx_aa_us;    /* TIMER0 at EVENTS_ADDRESS */
static uint32_t last_rx_ready_us; /* TIMER0 at EVENTS_READY  */

/*===========================================================================
 * 辅助函数
 *===========================================================================*/
static uint8_t count_ones(const uint8_t *data, uint8_t len)
{
	uint8_t count = 0;

	for (uint8_t i = 0; i < len; i++) {
		uint8_t byte = data[i];

		while (byte) {
			count += byte & 1;
			byte >>= 1;
		}
	}
	return count;
}

/*===========================================================================
 * 构造 ADV_IND PDU
 *===========================================================================*/
static void build_adv_ind_pdu(void)
{
	memset(&pdu_adv_ind, 0, sizeof(pdu_adv_ind));

	pdu_adv_ind.type    = PDU_ADV_TYPE_ADV_IND;
	pdu_adv_ind.tx_addr = 1;
	pdu_adv_ind.chan_sel = 0;
	pdu_adv_ind.rx_addr = 0;
	pdu_adv_ind.len     = BDADDR_SIZE + ADV_DATA_LEN;

	memcpy(pdu_adv_ind.adv_ind.addr, adv_addr, BDADDR_SIZE);
	memcpy(pdu_adv_ind.adv_ind.data, adv_data, ADV_DATA_LEN);
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
 * 启动 RTC0 — 为 radio_tmr_start 提供 32768 Hz 时基
 *
 * HAL 的 radio_tmr_start 使用 RTC0 CC[2] + PPI 在精确的 tick 触发
 * TIMER0 START → RXEN/TXEN。如果 RTC0 未运行, Counter 始终为 0,
 * CC[2] 永远不会匹配, PPI 永远不会触发, 导致 Radio 死循环。
 *
 * 在 Zephyr BLE Controller 中, 这由 ticker 子系统的 cntr_init() 完成。
 * 我们直接使用 HAL, 所以需要手动启动。
 *===========================================================================*/
static void rtc0_start(void)
{
	NRF_RTC0->TASKS_STOP = 1;
	NRF_RTC0->PRESCALER = 0;     /* 32768 Hz, 无分频 */
	NRF_RTC0->TASKS_CLEAR = 1;
	NRF_RTC0->TASKS_START = 1;

	printk("RTC0 started (32768 Hz)\n");
}

/*===========================================================================
 * 确保 Radio 回到 DISABLED 状态 (含 SW TIFS 清理)
 *===========================================================================*/
static void radio_ensure_disabled(void)
{
	NRF_RADIO->SHORTS = 0;

	/* 禁用自动切换 PPI, 防止 Timer CC 匹配后触发 RXEN/TXEN */
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN) | BIT(PPI_CH_TXEN));

	/* 停止 Timer, 防止 CC 事件在空闲期间持续触发 */
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_STOP);

	if (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled) {
		NRF_RADIO->EVENTS_DISABLED = 0;
		NRF_RADIO->TASKS_DISABLE = 1;
		while (NRF_RADIO->EVENTS_DISABLED == 0) {
		}
	}
}

/*===========================================================================
 * 配置 Radio — 广播态
 *===========================================================================*/
static void radio_configure_adv(void)
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

	/* SW TIFS: 不使用 TIFS 寄存器, 由 TIMER1 + PPI 控制 */
	NRF_RADIO->TIFS = 0;
}

/*===========================================================================
 * 配置 Radio — 连接态
 *===========================================================================*/
static void radio_configure_conn(void)
{
	radio_phy_set(0, 0);
	radio_tx_power_set(0);

	radio_pkt_configure(RADIO_PKT_CONF_LENGTH_8BIT,
			    PDU_DC_PAYLOAD_SIZE_MIN,
			    RADIO_PKT_CONF_FLAGS(
				    RADIO_PKT_CONF_PDU_TYPE_DC,
				    RADIO_PKT_CONF_PHY_LEGACY,
				    RADIO_PKT_CONF_CTE_DISABLED));

	radio_aa_set(conn_params.access_addr);

	uint32_t crc_iv = conn_params.crc_init[0] |
			  ((uint32_t)conn_params.crc_init[1] << 8) |
			  ((uint32_t)conn_params.crc_init[2] << 16);
	radio_crc_configure(PDU_CRC_POLYNOMIAL, crc_iv);

	/* SW TIFS: 不使用 TIFS 寄存器, 由 TIMER1 + PPI 控制 */
	NRF_RADIO->TIFS = 0;
}

/*===========================================================================
 * 配置 SW_SWITCH_TIMER (NRF_TIMER1)
 *
 * 对应真实 Controller 中 !CONFIG_BT_CTLR_TIFS_HW 路径:
 *   SW_SWITCH_TIMER = NRF_TIMER1, 1MHz, 16-bit
 *===========================================================================*/
static void sw_switch_timer_configure(void)
{
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_STOP);
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);

	NRF_TIMER1->INTENCLR = 0xFFFFFFFFUL;

	for (int i = 0; i < 6; i++) {
		NRF_TIMER1->EVENTS_COMPARE[i] = 0;
	}

	NRF_TIMER1->MODE      = TIMER_MODE_MODE_Timer;
	NRF_TIMER1->PRESCALER = 4;   /* 1MHz (16MHz / 2^4) */
	NRF_TIMER1->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;

	printk("SW_SWITCH_TIMER (TIMER1) configured: 1MHz, 16-bit\n");
}

/*===========================================================================
 * 配置 PPI 通道
 *
 * PPI CH14: RADIO END   → TIMER1 CLEAR  (包结束时清零定时器)
 * PPI CH15: TIMER1 CC[0] → RADIO RXEN   (定时触发 RX, TX→RX)
 * PPI CH16: TIMER1 CC[1] → RADIO TXEN   (定时触发 TX, RX→TX)
 *===========================================================================*/
static void ppi_configure(void)
{
	nrf_ppi_channel_endpoint_setup(
		NRF_PPI,
		(nrf_ppi_channel_t)PPI_CH_TIMER_CLEAR,
		(uint32_t)&NRF_RADIO->EVENTS_END,
		(uint32_t)&NRF_TIMER1->TASKS_CLEAR);

	nrf_ppi_channel_endpoint_setup(
		NRF_PPI,
		(nrf_ppi_channel_t)PPI_CH_RXEN,
		(uint32_t)&NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN],
		(uint32_t)&NRF_RADIO->TASKS_RXEN);

	nrf_ppi_channel_endpoint_setup(
		NRF_PPI,
		(nrf_ppi_channel_t)PPI_CH_TXEN,
		(uint32_t)&NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN],
		(uint32_t)&NRF_RADIO->TASKS_TXEN);

	/* Timer Clear PPI 常开, 其他按需启用 */
	nrf_ppi_channels_enable(NRF_PPI, BIT(PPI_CH_TIMER_CLEAR));
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN) | BIT(PPI_CH_TXEN));

	printk("PPI configured: CH%d END→CLEAR, CH%d CC[%d]→RXEN, CH%d CC[%d]→TXEN\n",
	       PPI_CH_TIMER_CLEAR, PPI_CH_RXEN, CC_IDX_RXEN, PPI_CH_TXEN, CC_IDX_TXEN);
}

/*===========================================================================
 * 数据信道频率设置
 *===========================================================================*/
static void data_chan_set(uint8_t chan)
{
	uint32_t freq;

	if (chan <= 10) {
		freq = 4 + (chan * 2U);
	} else if (chan <= 36) {
		freq = 28 + ((chan - 11) * 2U);
	} else {
		return;
	}

	radio_freq_chan_set(freq);
	radio_whiten_iv_set(chan);
}

/*===========================================================================
 * Channel Selection Algorithm #1
 *===========================================================================*/
static uint8_t chan_sel_1(void)
{
	uint8_t unmapped;

	unmapped = (last_unmapped_chan + conn_params.hop) % 37;
	last_unmapped_chan = unmapped;

	if (conn_params.chan_map[unmapped >> 3] & (1U << (unmapped & 7))) {
		return unmapped;
	}

	uint8_t remap_index = unmapped % conn_params.chan_count;
	uint8_t count = 0;

	for (uint8_t i = 0; i < 37; i++) {
		if (conn_params.chan_map[i >> 3] & (1U << (i & 7))) {
			if (count == remap_index) {
				return i;
			}
			count++;
		}
	}

	return 0;
}

/*===========================================================================
 * 验证 CONNECT_IND
 *===========================================================================*/
static bool validate_connect_ind(const struct pdu_adv *rx_pdu)
{
	if (rx_pdu->type != PDU_ADV_TYPE_CONNECT_IND) {
		return false;
	}
	if (rx_pdu->len != sizeof(struct pdu_adv_connect_ind)) {
		return false;
	}
	if (memcmp(rx_pdu->connect_ind.adv_addr, adv_addr, BDADDR_SIZE) != 0) {
		return false;
	}
	return true;
}

/*===========================================================================
 * 解析 CONNECT_IND
 *===========================================================================*/
static void parse_connect_ind(const struct pdu_adv *rx_pdu)
{
	const struct pdu_adv_connect_ind *ci = &rx_pdu->connect_ind;

	memcpy(conn_params.peer_addr, ci->init_addr, BDADDR_SIZE);
	memcpy(conn_params.access_addr, ci->access_addr, 4);
	memcpy(conn_params.crc_init, ci->crc_init, 3);

	conn_params.win_size   = ci->win_size;
	conn_params.win_offset = sys_le16_to_cpu(ci->win_offset);
	conn_params.interval   = sys_le16_to_cpu(ci->interval);
	conn_params.latency    = sys_le16_to_cpu(ci->latency);
	conn_params.timeout    = sys_le16_to_cpu(ci->timeout);

	memcpy(conn_params.chan_map, ci->chan_map, PDU_CHANNEL_MAP_SIZE);
	conn_params.hop = ci->hop;
	conn_params.sca = ci->sca;
	conn_params.chan_count = count_ones(conn_params.chan_map,
					    PDU_CHANNEL_MAP_SIZE);
}

/*===========================================================================
 * 打印 CONNECT_IND 参数 (延迟调用)
 *===========================================================================*/
static void print_connect_ind(void)
{
	uint32_t aa_val = sys_get_le32(conn_params.access_addr);
	uint32_t crc_val = conn_params.crc_init[0] |
			   ((uint32_t)conn_params.crc_init[1] << 8) |
			   ((uint32_t)conn_params.crc_init[2] << 16);

	printk("\n========== CONNECT_IND Received ==========\n");
	printk("  Peer:       %02X:%02X:%02X:%02X:%02X:%02X\n",
	       conn_params.peer_addr[5], conn_params.peer_addr[4],
	       conn_params.peer_addr[3], conn_params.peer_addr[2],
	       conn_params.peer_addr[1], conn_params.peer_addr[0]);
	printk("  AA:         0x%08X\n", aa_val);
	printk("  CRC Init:   0x%06X\n", crc_val);
	printk("  WinSize:    %d (x1.25ms = %d us)\n",
	       conn_params.win_size,
	       (uint32_t)conn_params.win_size * CONN_INT_UNIT_US);
	printk("  WinOffset:  %d (x1.25ms = %d us)\n",
	       conn_params.win_offset,
	       (uint32_t)conn_params.win_offset * CONN_INT_UNIT_US);
	printk("  Interval:   %d (x1.25ms = %d us)\n",
	       conn_params.interval,
	       (uint32_t)conn_params.interval * CONN_INT_UNIT_US);
	printk("  Latency:    %d\n", conn_params.latency);
	printk("  Timeout:    %d (x10ms)\n", conn_params.timeout);
	printk("  Hop:        %d\n", conn_params.hop);
	printk("  SCA:        %d (%d ppm)\n", conn_params.sca,
	       sca_ppm_table[conn_params.sca & 0x07]);
	printk("  Chan Count: %d / 37\n", conn_params.chan_count);
	printk("==========================================\n");
}

/*===========================================================================
 * 打印锚点计算的详细过程 (本 Demo 核心教学输出)
 *===========================================================================*/
static void print_anchor_calculation(uint32_t conn_offset_us,
				     uint32_t offset_ticks,
				     uint32_t offset_remainder_ps,
				     uint32_t interval_ticks,
				     uint32_t interval_remainder_ps,
				     uint16_t master_sca_ppm,
				     uint32_t combined_sca_ppm,
				     uint32_t ww_periodic_us,
				     uint32_t conn_interval_us)
{
	printk("\n===== Anchor Point Calculation =====\n");
	printk("\n[Step 1] T0: CONNECT_IND end time\n");
	printk("  connect_end_rtc = %u (RTC0 ticks, 32768 Hz)\n",
	       connect_end_rtc);
	printk("  T0 = %u x 30.52us = ~%u us\n",
	       connect_end_rtc,
	       HAL_TICKER_TICKS_TO_US(connect_end_rtc));

	printk("\n[Step 2] Calculate first anchor offset\n");
	printk("  BLE Spec formula:\n");
	printk("    A0 = T0 + winOffset*1.25ms + transmitWindowDelay\n");
	printk("  Zephyr implementation adds margins:\n");
	printk("    conn_offset_us = winOffset*1250 + WIN_DELAY(1250)\n");
	printk("                   - TICKER_RES_MARGIN(%d)\n",
	       EVENT_TICKER_RES_MARGIN_US);
	printk("                   - EVENT_JITTER(%d)\n",
	       EVENT_JITTER_US);
	printk("                   - rx_ready_delay(%d)\n",
	       radio_rx_ready_delay_get(0, 0));
	printk("                   - ww_periodic(%d)\n", ww_periodic_us);
	printk("    = %d*1250 + 1250 - %d - %d - %d - %d\n",
	       conn_params.win_offset,
	       EVENT_TICKER_RES_MARGIN_US,
	       EVENT_JITTER_US,
	       radio_rx_ready_delay_get(0, 0),
	       ww_periodic_us);
	printk("    = %u us\n", conn_offset_us);

	printk("\n[Step 3] Convert to RTC ticks + remainder\n");
	printk("  RTC runs at 32768 Hz, 1 tick = 30.517578125 us\n");
	printk("  offset_ticks     = %u\n", offset_ticks);
	printk("  offset_remainder = %u ps (sub-tick precision)\n",
	       offset_remainder_ps);

	printk("\n[Step 4] First anchor point (absolute RTC tick)\n");
	printk("  initial_anchor = connect_end_rtc + offset_ticks\n");
	printk("                 = %u + %u = %u\n",
	       connect_end_rtc, offset_ticks,
	       anchor.initial_anchor_rtc);

	printk("\n[Step 5] connInterval in RTC ticks\n");
	printk("  interval = %u us\n", conn_interval_us);
	printk("  interval_ticks     = %u\n", interval_ticks);
	printk("  interval_remainder = %u ps\n", interval_remainder_ps);
	printk("  (remainder accumulates each event, carries +1 tick\n");
	printk("   when >= 1 tick period, preventing drift)\n");

	printk("\n[Step 6] Window Widening parameters\n");
	printk("  Master SCA: %d ppm\n", master_sca_ppm);
	printk("  Local SCA:  %d ppm\n", LOCAL_SCA_PPM);
	printk("  Combined:   %d ppm\n", combined_sca_ppm);
	printk("  WW periodic = ceil(%d * %d / 1000000) = %d us/event\n",
	       combined_sca_ppm, conn_interval_us, ww_periodic_us);
	printk("  WW max      = interval/2 - T_IFS = %d us\n",
	       anchor.ww_max_us);

	printk("\n===== Anchor Sequence Preview =====\n");
	printk("  A0 (first)  : RTC tick %u\n", anchor.initial_anchor_rtc);
	uint32_t a1 = (anchor.initial_anchor_rtc + interval_ticks) &
		      HAL_TICKER_CNTR_MASK;
	uint32_t a2 = (a1 + interval_ticks) & HAL_TICKER_CNTR_MASK;
	printk("  A1 (A0+intv) : RTC tick %u (delta=%u ticks)\n",
	       a1, interval_ticks);
	printk("  A2 (A1+intv) : RTC tick %u (delta=%u ticks)\n",
	       a2, interval_ticks);
	printk("  (remainder carry may add +1 tick per event)\n");
	printk("=====================================\n\n");
}

/*===========================================================================
 * 构造空 PDU
 *===========================================================================*/
static void build_empty_pdu(struct pdu_data *pdu)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = PDU_DATA_LLID_DATA_CONTINUE;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = 0;
	pdu->len   = 0;
}

/*===========================================================================
 * 构造 LL_VERSION_IND
 *===========================================================================*/
static void build_version_ind(struct pdu_data *pdu)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = 0;
	pdu->len   = 1 + sizeof(struct pdu_data_llctrl_version_ind);

	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_VERSION_IND;
	pdu->llctrl.version_ind.version_number = 0x0C;
	pdu->llctrl.version_ind.company_id = sys_cpu_to_le16(0xFFFF);
	pdu->llctrl.version_ind.sub_version_number = sys_cpu_to_le16(0x0001);
}

/*===========================================================================
 * 构造 LL_FEATURE_RSP
 *===========================================================================*/
static void build_feature_rsp(struct pdu_data *pdu)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = 0;
	pdu->len   = 1 + sizeof(struct pdu_data_llctrl_feature_rsp);

	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_FEATURE_RSP;
	memset(pdu->llctrl.feature_rsp.features, 0, 8);
}

/*===========================================================================
 * 构造 LL_UNKNOWN_RSP
 *===========================================================================*/
static void build_unknown_rsp(struct pdu_data *pdu, uint8_t unknown_type)
{
	memset(pdu, 0, sizeof(*pdu));
	pdu->ll_id = PDU_DATA_LLID_CTRL;
	pdu->nesn  = rx_nesn;
	pdu->sn    = tx_sn;
	pdu->md    = 0;
	pdu->len   = 1 + sizeof(struct pdu_data_llctrl_unknown_rsp);

	pdu->llctrl.opcode = PDU_DATA_LLCTRL_TYPE_UNKNOWN_RSP;
	pdu->llctrl.unknown_rsp.type = unknown_type;
}

/*===========================================================================
 * LL Control PDU 处理
 *===========================================================================*/
static bool handle_ll_control(const struct pdu_data *rx_pdu)
{
	uint8_t opcode = rx_pdu->llctrl.opcode;

	switch (opcode) {
	case PDU_DATA_LLCTRL_TYPE_VERSION_IND:
		build_version_ind(&tx_pdu_buf);
		return true;

	case PDU_DATA_LLCTRL_TYPE_FEATURE_REQ:
	case PDU_DATA_LLCTRL_TYPE_PER_INIT_FEAT_XCHG:
		build_feature_rsp(&tx_pdu_buf);
		return true;

	case PDU_DATA_LLCTRL_TYPE_TERMINATE_IND:
		printk("[ANCHOR] RX: LL_TERMINATE_IND\n");
		conn_terminated = true;
		return false;

	case PDU_DATA_LLCTRL_TYPE_PING_REQ:
		memset(&tx_pdu_buf, 0, sizeof(tx_pdu_buf));
		tx_pdu_buf.ll_id = PDU_DATA_LLID_CTRL;
		tx_pdu_buf.nesn  = rx_nesn;
		tx_pdu_buf.sn    = tx_sn;
		tx_pdu_buf.len   = 1;
		tx_pdu_buf.llctrl.opcode = PDU_DATA_LLCTRL_TYPE_PING_RSP;
		return true;

	default:
		build_unknown_rsp(&tx_pdu_buf, opcode);
		return true;
	}
}

/*===========================================================================
 * 广播信道处理
 *===========================================================================*/
static int adv_on_channel(uint32_t channel)
{
	uint32_t freq;

	switch (channel) {
	case 37: freq = 2;  break;
	case 38: freq = 26; break;
	case 39: freq = 80; break;
	default: return 0;
	}

	radio_freq_chan_set(freq);
	radio_whiten_iv_set(channel);

	NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;

	/*==============================================================
	 * 第 1 阶段: TX ADV_IND (SW TIFS)
	 *
	 * PPI_CH_RXEN 在 TX 期间保持禁用!
	 * TX END 时 PPI_CH_TIMER_CLEAR 清零 Timer1。
	 * TX END 后启用 PPI_CH_RXEN, 110μs 后 RXEN 触发。
	 *==============================================================*/

	radio_pkt_tx_set(&pdu_adv_ind);

	radio_status_reset();
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_ADDRESS = 0;

	/* SW TIFS: 不使用 DISABLED_RXEN SHORT */
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;

	nrf_timer_cc_set(NRF_TIMER1, (nrf_timer_cc_channel_t)CC_IDX_RXEN,
			 T_IFS_US - RX_RAMP_US);
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN) | BIT(PPI_CH_TXEN));

	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_START);

	NRF_RADIO->TASKS_TXEN = 1;

	/* 等待 TX END */
	while (NRF_RADIO->EVENTS_END == 0) {
	}
	NRF_RADIO->EVENTS_END = 0;

	/*==============================================================
	 * TX→RX 转换 (SW TIFS)
	 *
	 * TX END 时 PPI 清零 Timer1, 从 0 开始计数。
	 * 清除陈旧 COMPARE 事件, 切换到 RX 缓冲区, 启用 PPI_CH_RXEN。
	 * 110μs 后 CC[0] 触发 RXEN → 40μs ramp → RX START (T_IFS=150μs)
	 *==============================================================*/

	NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN] = 0;
	(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN];

	radio_pkt_rx_set(&pdu_adv_rx);

	nrf_ppi_channels_enable(NRF_PPI, BIT(PPI_CH_RXEN));

	/* 等待 TX DISABLED */
	while (NRF_RADIO->EVENTS_DISABLED == 0) {
	}
	NRF_RADIO->EVENTS_DISABLED = 0;

	/*==============================================================
	 * 第 2 阶段: RX 等待 CONNECT_IND
	 *
	 * ★ 必须清除 TX 期间的 ADDRESS 事件!
	 * EVENTS_ADDRESS 在 TX 时被硬件置 1 (AA 发送到空中时),
	 * 若不清除, 后续逻辑会误判为已收到包。
	 *==============================================================*/

	NRF_RADIO->EVENTS_ADDRESS = 0;

	uint32_t timeout_loops = ADV_RX_TIMEOUT_US * 16;
	bool rx_done = false;

	while (timeout_loops > 0) {
		if (NRF_RADIO->EVENTS_DISABLED) {
			rx_done = true;
			break;
		}
		timeout_loops--;
	}

	/* ★ 关键时刻: 立即捕获 T0 — CONNECT_IND 结束时的 RTC0 tick
	 *
	 * 必须在 CRC 检查和 validate 之前读取, 减少软件延迟!
	 * 这个值是整个锚点体系的基准, 所有后续连接事件
	 * 的定时都从这个 T0 开始推算。
	 *
	 * 在 Zephyr 控制器中:
	 *   ftr->ticks_anchor = radio_tmr_start_get() */
	uint32_t rx_end_rtc = NRF_RTC0->COUNTER & HAL_TICKER_CNTR_MASK;

	if (!rx_done) {
		radio_ensure_disabled();
		return 0;
	}

	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN));
	NRF_RADIO->SHORTS = 0;

	if (!radio_crc_is_valid()) {
		return 0;
	}

	if (validate_connect_ind(&pdu_adv_rx)) {
		connect_end_rtc = rx_end_rtc;
		parse_connect_ind(&pdu_adv_rx);
		return 1;
	}

	return 0;
}

/*===========================================================================
 * 广播事件
 *===========================================================================*/
static int adv_event(void)
{
	static const uint32_t adv_channels[] = {37, 38, 39};

	for (int i = 0; i < 3; i++) {
		int ret = adv_on_channel(adv_channels[i]);

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

/*===========================================================================
 * 单个连接事件 (使用 HAL 硬件定时 + SW TIFS)
 *
 * ★ 锚点相关的核心操作:
 *
 *   radio_tmr_start(0, ticks_at_start, remainder_ps)
 *     → RTC0 CC + PPI + TIMER0 在精确的锚点时刻启动 Radio RX
 *
 * ★ SW TIFS RX→TX 转换:
 *
 *   RX END → PPI 清零 Timer1
 *   Timer1 CC[1]=110μs → PPI 触发 TXEN
 *   TXEN → 40μs ramp → TX START (T_IFS=150μs)
 *===========================================================================*/
static bool conn_event(uint32_t ticks_at_start, uint32_t remainder_ps,
		       uint32_t hcto_add)
{
	/* 检查目标 RTC tick 是否在未来 */
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

	/* ★ 清理上一个事件遗留的 HAL PPI/Timer 状态
	 *   radio_tmr_status_reset() 禁用所有 HAL radio PPI 通道 (CH6/7/20-27),
	 *   并禁用 RTC0 CC[2] 事件产生。
	 *   这防止遗留的 HCTO 或 RXEN/TXEN PPI 干扰新事件。 */
	radio_tmr_status_reset();

	/* SW TIFS: 不使用 DISABLED_TXEN SHORT, RX→TX 由 Timer1+PPI 驱动 */
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;

	/* 确保 PPI 切换通道禁用 */
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_RXEN) | BIT(PPI_CH_TXEN));

	/* 设置 Timer1 CC[1] for RX→TX 转换
	 * CC = T_IFS - TX_RAMP - RX_CHAIN_DELAY = 150 - 40 - 10 = 100μs
	 * 补偿 RX chain delay: EVENTS_END 比实际 air-time 结束晚 ~9.4μs,
	 * 所以 CC 值需要减去这个延迟以保证 T_IFS=150±2μs */
	nrf_timer_cc_set(NRF_TIMER1, (nrf_timer_cc_channel_t)CC_IDX_TXEN,
			 T_IFS_US - TX_RAMP_US - RX_CHAIN_DELAY_1M_US);

	/* 启动 Timer1 */
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_START);

	radio_pkt_rx_set(&pdu_data_rx);

	/* ★ 关键: 每次调用 radio_tmr_start 前必须确保硬件状态干净
	 *
	 *   1. 停止 TIMER0: radio_tmr_start 只做 CLEAR 不做 STOP,
	 *      如果 TIMER0 仍在运行, CLEAR 后计数器从 0 重新开始,
	 *      CC[0]=remainder_us 会立即触发 RXEN (不等 RTC 锚点)
	 *
	 *   2. 清除 RTC0 EVENTS_COMPARE[2]: PPI 是边沿触发 (0→1),
	 *      上一个事件留下的锁存值 =1 会阻止新的比较匹配触发 PPI,
	 *      导致 TIMER0 永远不会被 START → RXEN/HCTO 都不会触发
	 *      → while(EVENTS_DISABLED==0) 死循环 */
	nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_STOP);
	NRF_RTC0->EVENTS_COMPARE[2] = 0;

	/* ★ 这里将锚点 tick 传给硬件: radio_tmr_start 内部配置
	 *    RTC0 CC + PPI + TIMER0, 在精确的锚点时刻启动 Radio RX */
	uint32_t remainder_us = radio_tmr_start(0, ticks_at_start,
						remainder_ps);

	radio_tmr_aa_capture();
	radio_tmr_aa_save(0);

	/* HCTO: Header-Complete TimeOut
	 * 保护机制: 如果在锚点 + HCTO 时间内没收到包头, 自动 DISABLE Radio */
	uint32_t hcto = remainder_us + hcto_add +
			radio_rx_ready_delay_get(0, 0) +
			ADDR_US_1M +
			radio_rx_chain_delay_get(0, 0);
	radio_tmr_hcto_configure(hcto);

	radio_tmr_end_capture();

	/* 等待: 要么收到包 (END→DISABLE), 要么 HCTO 超时 (COMPARE→DISABLE) */
	{
		uint32_t safety = 0;

		while (NRF_RADIO->EVENTS_DISABLED == 0) {
			if (++safety > 5000000) {
				printk("!STUCK RX! ST=%u EVCC2=%u T0=%u\n",
				       NRF_RADIO->STATE,
				       NRF_RTC0->EVENTS_COMPARE[2],
				       NRF_TIMER0->EVENTS_COMPARE[1]);
				NRF_RADIO->TASKS_DISABLE = 1;
				while (NRF_RADIO->EVENTS_DISABLED == 0) {
				}
				conn_event_rx_timeout++;
				conn_event_counter++;
				return false;
			}
		}
	}
	NRF_RADIO->EVENTS_DISABLED = 0;

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

	/* ★ Save TIMER0 captures BEFORE TX phase
	 * TX EVENTS_READY would overwrite CC[TRX_CC_OFFSET] via PPI */
	last_rx_aa_us = radio_tmr_aa_get();
	last_rx_ready_us = radio_tmr_ready_get();

	/*==============================================================
	 * SW TIFS: RX→TX 转换
	 *
	 * RX END 已触发:
	 *   - PPI_CH_TIMER_CLEAR: Timer1 清零到 0, 继续计数
	 *   - SHORTS: END→DISABLE → Radio DISABLED
	 *
	 * 清除 EVENTS_COMPARE (写后读回确保 APB 生效),
	 * 处理数据, 准备 TX PDU,
	 * 然后启用 PPI_CH_TXEN: Timer1 CC[1]=110μs 时触发 TXEN。
	 *==============================================================*/

	NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN] = 0;
	(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN];
	NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN] = 0;
	(void)NRF_TIMER1->EVENTS_COMPARE[CC_IDX_RXEN];

	/* SN/NESN 流控 */
	if (pdu_data_rx.nesn != tx_sn) {
		tx_sn = pdu_data_rx.nesn;
	}
	bool new_packet = false;

	if (pdu_data_rx.sn == rx_nesn) {
		new_packet = true;
		rx_nesn ^= 1;
	}

	/* LL Control 处理 */
	tx_pdu_pending = false;
	if (new_packet && pdu_data_rx.len > 0) {
		if (pdu_data_rx.ll_id == PDU_DATA_LLID_CTRL) {
			tx_pdu_pending = handle_ll_control(&pdu_data_rx);
		}
	}

	/* 准备 TX 包 */
	if (tx_pdu_pending) {
		tx_pdu_buf.nesn = rx_nesn;
		tx_pdu_buf.sn   = tx_sn;
		radio_pkt_tx_set(&tx_pdu_buf);
	} else {
		build_empty_pdu(&tx_pdu_buf);
		radio_pkt_tx_set(&tx_pdu_buf);
	}

	/* 启用 PPI_CH_TXEN: Timer1 CC[1]=100μs 后触发 TXEN */

	/* TX 诊断: 记录 PPI 启用时 Timer1 值 (确认 < CC[1]) */
	nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CAPTURE3);
	uint32_t t1_at_ppi_en = nrf_timer_cc_get(NRF_TIMER1, NRF_TIMER_CC_CHANNEL3);
	uint32_t cc1_already = NRF_TIMER1->EVENTS_COMPARE[CC_IDX_TXEN];

	nrf_ppi_channels_enable(NRF_PPI, BIT(PPI_CH_TXEN));

	/* SHORTS 保持 READY_START | END_DISABLE (TX 阶段同样适用) */
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;

	/* 等待 TX 完成: PPI TXEN → READY_START → TX → END_DISABLE → DISABLED */
	{
		uint32_t safety = 0;

		while (NRF_RADIO->EVENTS_DISABLED == 0) {
			if (++safety > 2000000) {
				printk("!STUCK TX! ST=%u T1=%u CC1F=%u\n",
				       NRF_RADIO->STATE,
				       t1_at_ppi_en, cc1_already);
				NRF_RADIO->TASKS_DISABLE = 1;
				while (NRF_RADIO->EVENTS_DISABLED == 0) {
				}
				break;
			}
		}
	}

	/* TX 诊断: 确认 TX END 真的发生了 */
	uint32_t tx_end_fired = NRF_RADIO->EVENTS_END;

	NRF_RADIO->EVENTS_DISABLED = 0;

	/* 清理 */
	nrf_ppi_channels_disable(NRF_PPI, BIT(PPI_CH_TXEN));
	NRF_RADIO->SHORTS = 0;

	/* 前3个事件打印 TX 诊断 */
	if (conn_event_counter < 3) {
		printk("[TX_DBG] #%u T1@en=%u CC1_pre=%u END=%u "
		       "hdr=0x%02X len=%u\n",
		       conn_event_counter, t1_at_ppi_en,
		       cc1_already, tx_end_fired,
		       *(uint8_t *)&tx_pdu_buf, tx_pdu_buf.len);
	}

	if (tx_pdu_pending) {
		tx_sn ^= 1;
	}

	conn_event_counter++;
	return true;
}

/*===========================================================================
 * 连接态主循环 — 锚点机制的完整演示
 *
 * ★ 这是本 Demo 的核心: 展示锚点如何计算、递推、更新
 *
 * 锚点生命周期:
 *
 *   1. 初始化: 从 CONNECT_IND 计算首次锚点
 *   2. 递推:   每个 connInterval 推进一次 (ticks + remainder 累积)
 *   3. RX成功: 更新锚点 (重置 Window Widening)
 *   4. RX失败: 保持旧锚点, Window Widening 增长
 *   5. 超时:   supervision timeout → 断开连接
 *===========================================================================*/
static void conn_loop(void)
{
	/* 初始化连接状态 */
	tx_sn = 0;
	rx_nesn = 0;
	conn_event_counter = 0;
	last_unmapped_chan = 0;
	conn_terminated = false;
	tx_pdu_pending = false;
	conn_event_rx_ok = 0;
	conn_event_rx_timeout = 0;
	conn_event_rx_crc_err = 0;

	/* 初始化锚点追踪器 */
	memset(&anchor, 0, sizeof(anchor));

	radio_configure_conn();

	/* ==== 连接参数计算 ==== */
	uint32_t conn_interval_us = (uint32_t)conn_params.interval * CONN_INT_UNIT_US;
	uint32_t win_size_us = (uint32_t)conn_params.win_size * CONN_INT_UNIT_US;

	/* ★ Window Widening 参数计算
	 *
	 * BLE Core Spec Vol 6, Part B, 4.5.7:
	 *   windowWidening = ((masterSCA + slaveSCA) / 1000000) × timeSinceLastAnchor
	 *
	 * 每个 connInterval 的 WW 增量:
	 *   ww_periodic = ceil((master_ppm + slave_ppm) × interval_us / 1000000) */
	uint16_t master_sca_ppm = sca_ppm_table[conn_params.sca & 0x07];
	uint32_t combined_sca_ppm = (uint32_t)master_sca_ppm + LOCAL_SCA_PPM;
	uint32_t ww_periodic_us = DIV_ROUND_UP(
		combined_sca_ppm * conn_interval_us, 1000000);
	uint32_t ww_max_us = (conn_interval_us >> 1) - T_IFS_US;

	anchor.ww_periodic_us = ww_periodic_us;
	anchor.ww_max_us = ww_max_us;

	uint32_t ww_event_us = 0;
	uint32_t win_size_event_us = win_size_us;
	anchor.win_size_event_us = win_size_us;

	/* ==== ★ 首次锚点计算 ====
	 *
	 * 这是整个锚点体系的起点!
	 *
	 * BLE Spec 原始公式 (Vol 6, Part B, 4.5.1):
	 *   firstAnchor = CONNECT_IND_end + transmitWindowOffset + transmitWindowDelay
	 *
	 * Zephyr 实现 (ull_periph.c: ull_periph_setup) 添加了工程余量:
	 *   conn_offset_us = winOffset × 1250
	 *                  + WIN_DELAY_LEGACY (1250μs)
	 *                  - EVENT_TICKER_RES_MARGIN   (RTC 分辨率余量)
	 *                  - EVENT_JITTER              (时钟抖动余量)
	 *                  - radio_rx_ready_delay      (RX 启动提前量)
	 *                  - ww_periodic               (首次 WW)
	 *
	 * 减去这些值是为了让 Radio 提前打开 RX, 确保不会错过 Master 的包。
	 * 实际的 RX 窗口 = [anchor - margins, anchor + margins + ww + win_size]
	 */
	uint32_t conn_offset_us = (uint32_t)conn_params.win_offset * CONN_INT_UNIT_US
				+ WIN_DELAY_LEGACY
				- EVENT_TICKER_RES_MARGIN_US
				- EVENT_JITTER_US
				- radio_rx_ready_delay_get(0, 0)
				- ww_periodic_us;

	anchor.initial_anchor_us = conn_offset_us;

	/* ★ 关键: 把 μs 转换为 RTC ticks + 皮秒余量
	 *
	 * 为什么需要两部分?
	 *   RTC0 运行在 32768 Hz, 1 tick = 30517.578125 ns ≈ 30.52 μs
	 *   μs 级别的偏移量无法精确用整数 tick 表示
	 *   余量记录截断误差, 每次累积, 满一个 tick 时进位
	 *   这就是 Zephyr Ticker 的 "亚 tick 精度" 机制 */
	uint32_t offset_ticks = HAL_TICKER_US_TO_TICKS(conn_offset_us);
	uint32_t offset_remainder_ps = HAL_TICKER_REMAINDER(conn_offset_us);

	uint32_t interval_ticks = HAL_TICKER_US_TO_TICKS(conn_interval_us);
	uint32_t interval_remainder_ps = HAL_TICKER_REMAINDER(conn_interval_us);

	uint32_t ps_per_tick = HAL_TICKER_CNTR_CLK_UNIT_FSEC /
			       HAL_TICKER_FSEC_PER_PSEC;

	/* 首次锚点的绝对 RTC tick */
	uint32_t next_event_rtc = (connect_end_rtc + offset_ticks) &
				  HAL_TICKER_CNTR_MASK;
	uint32_t next_event_remainder_ps = offset_remainder_ps;

	anchor.initial_anchor_rtc = next_event_rtc;
	anchor.current_anchor_rtc = next_event_rtc;
	anchor.last_rx_anchor_rtc = connect_end_rtc;

	/* Supervision timeout */
	uint32_t supervision_timeout_us = (uint32_t)conn_params.timeout * 10000;
	uint32_t last_rx_rtc = connect_end_rtc;

	while (!conn_terminated) {
		/* ★ Window Widening 累积
		 *
		 * 每过一个 connInterval, WW 增加 ww_periodic:
		 *   ww = (masterSCA + slaveSCA) × N × connInterval
		 *
		 * 直到成功 RX 时重置为 0 */
		ww_event_us += ww_periodic_us;
		if (ww_event_us > ww_max_us) {
			ww_event_us = ww_max_us;
		}
		anchor.ww_current_us = ww_event_us;

		/* HCTO 窗口 = 2×(jitter + margin + ww) + win_size
		 * 对称地向两侧扩展, 所以乘 2 */
		uint32_t hcto_add = ((EVENT_JITTER_US +
				      EVENT_TICKER_RES_MARGIN_US +
				      ww_event_us) << 1) +
				    win_size_event_us;

		/* 记录当前锚点 */
		anchor.current_anchor_rtc = next_event_rtc;

		/* ==== 执行连接事件 ==== */
		bool rx_ok = conn_event(next_event_rtc,
					next_event_remainder_ps,
					hcto_add);

		int32_t drift_us_log = 0;

		if (rx_ok) {
			/* ★ 锚点漂移校正 (Drift Correction)
			 *
			 * 利用 TIMER0 在 EVENTS_ADDRESS 和 EVENTS_READY 的
			 * 硬件捕获值, 计算 Master 实际发送时刻与我们预期的偏差,
			 * 然后修正锚点, 防止时钟漂移跨事件累积。
			 *
			 * 与 Zephyr ULL ull_drift_ticks_get() 完全相同的算法:
			 *   actual = aa_us - ready_us  (READY→ADDRESS 实际时间)
			 *   expected_center = JIT + MARGIN + ADDR_1M
			 *   drift = actual - expected_center
			 */
			uint32_t actual_s2a = last_rx_aa_us - last_rx_ready_us;
			int32_t drift_us = (int32_t)actual_s2a -
				(int32_t)(EVENT_JITTER_US +
					  EVENT_TICKER_RES_MARGIN_US +
					  ADDR_US_1M);
			drift_us_log = drift_us;

			/* 将 drift 转换为皮秒并修正 remainder */
			int64_t adj_ps = (int64_t)next_event_remainder_ps +
					 (int64_t)drift_us * 1000000LL;

			while (adj_ps >= (int64_t)ps_per_tick) {
				adj_ps -= ps_per_tick;
				next_event_rtc = (next_event_rtc + 1) &
						 HAL_TICKER_CNTR_MASK;
			}
			while (adj_ps < 0) {
				adj_ps += ps_per_tick;
				next_event_rtc = (next_event_rtc - 1) &
						 HAL_TICKER_CNTR_MASK;
			}
			next_event_remainder_ps = (uint32_t)adj_ps;

			/* ★ 锚点更新 (Anchor Point Update)
			 *
			 * 漂移校正后, 用校正后的 tick 作为新参考锚点。
			 * 后续事件从这个校正后的锚点推算,
			 * 时钟漂移误差不会跨事件累积! */
			last_rx_rtc = next_event_rtc;
			anchor.last_rx_anchor_rtc = next_event_rtc;
			anchor.anchor_update_count++;
			anchor.consecutive_misses = 0;

			ww_event_us = 0;
			win_size_event_us = 0;
			anchor.ww_current_us = 0;
			anchor.win_size_event_us = 0;
		} else {
			/* ★ 锚点未更新 — Window Widening 继续增长
			 *
			 * 丢包意味着我们不知道 Master 的精确时刻,
			 * 只能从上一个已知锚点推算。
			 * WW 每丢一个包增长一次, RX 窗口越来越宽。 */
			anchor.consecutive_misses++;
			if (anchor.consecutive_misses > anchor.max_consecutive_misses) {
				anchor.max_consecutive_misses = anchor.consecutive_misses;
			}

			/* Supervision timeout 检测 */
			uint32_t elapsed_ticks = (next_event_rtc - last_rx_rtc) &
						 HAL_TICKER_CNTR_MASK;
			uint32_t elapsed_us = HAL_TICKER_TICKS_TO_US(elapsed_ticks);

			if (elapsed_us >= supervision_timeout_us) {
				printk("[ANCHOR] Supervision timeout! no RX for %u us\n",
				       elapsed_us);
				break;
			}
		}

		/* ★ 锚点递推: 推进到下一个 connInterval
		 *
		 * next_anchor = current_anchor + interval_ticks
		 *
		 * 关键细节: 皮秒余量累积与进位
		 *   remainder 每次 += interval_remainder
		 *   当 remainder >= 1 tick 的皮秒数时, 进位 +1 tick
		 *   这防止了舍入误差的长期累积
		 *
		 * 举例 (connInterval = 30ms):
		 *   30000 μs / 30.52 μs = 982.97 ticks
		 *   ticks_interval = 982
		 *   remainder 追踪 0.97 × 30.52μs ≈ 29.6 μs
		 *   每 ~33 个事件 remainder 溢出, 多加 1 tick
		 *   长期平均: 精确等于 30000 μs */
		next_event_rtc = (next_event_rtc + interval_ticks) &
				 HAL_TICKER_CNTR_MASK;
		next_event_remainder_ps += interval_remainder_ps;
		if (next_event_remainder_ps >= ps_per_tick) {
			next_event_remainder_ps -= ps_per_tick;
			next_event_rtc = (next_event_rtc + 1) &
					 HAL_TICKER_CNTR_MASK;
		}

		uint32_t current_event = conn_event_counter;

		/* 锚点状态日志:
		 * 仅打印前 5 个事件和出错事件, 避免 UART 阻塞 */
		if (current_event <= 5 ||
		    anchor.consecutive_misses > 3) {
			printk("[ANCHOR] #%u ch=%u %s | "
			       "rtc=%u ww=%u drift=%d | "
			       "rx=%u/%u miss=%u\n",
			       current_event,
			       last_unmapped_chan,
			       rx_ok ? "OK" : "MISS",
			       anchor.current_anchor_rtc,
			       anchor.ww_current_us,
			       drift_us_log,
			       conn_event_rx_ok,
			       current_event,
			       anchor.consecutive_misses);
		}
	}

	/* 清理 */
	NRF_RADIO->SHORTS = 0;
	NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;

	printk("\n========== Connection Summary ==========\n");
	printk("  Total events:   %u\n", conn_event_counter);
	printk("  RX OK:          %u\n", conn_event_rx_ok);
	printk("  RX timeout:     %u\n", conn_event_rx_timeout);
	printk("  RX CRC error:   %u\n", conn_event_rx_crc_err);
	printk("  Anchor updates: %u\n", anchor.anchor_update_count);
	printk("  Max consecutive misses: %u\n", anchor.max_consecutive_misses);
	printk("=========================================\n");

	/* 连接参数和锚点计算详情 (连接结束后打印, 避免影响活跃连接) */
	printk("[STATE] Connection state details:\n");
	print_connect_ind();
	print_anchor_calculation(conn_offset_us,
				 offset_ticks,
				 offset_remainder_ps,
				 interval_ticks,
				 interval_remainder_ps,
				 master_sca_ppm,
				 combined_sca_ppm,
				 ww_periodic_us,
				 conn_interval_us);
}

/*===========================================================================
 * 主函数
 *===========================================================================*/
int main(void)
{
	printk("\n");
	printk("===================================================\n");
	printk("  BLE PHY Anchor Point Demo\n");
	printk("  演示 BLE 连接中的锚点 (Anchor) 机制\n");
	printk("===================================================\n");
	printk("Device: %s  Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
	       ADV_NAME,
	       adv_addr[5], adv_addr[4], adv_addr[3],
	       adv_addr[2], adv_addr[1], adv_addr[0]);
	printk("\n");
	printk("Anchor Point 生命周期:\n");
	printk("  1. CONNECT_IND → 计算首次锚点 (T0 + offset)\n");
	printk("  2. 每个 connInterval 递推 (ticks + remainder)\n");
	printk("  3. RX 成功 → 更新锚点, WW 重置\n");
	printk("  4. RX 失败 → WW 增长, 窗口加宽\n");
	printk("  5. Supervision timeout → 断开\n");
	printk("\n");

	hfclk_start();
	rtc0_start();
	sw_switch_timer_configure();
	ppi_configure();

	while (1) {
		printk("[STATE] Entering ADVERTISING state\n");
		adv_event_count = 0;

		radio_configure_adv();
		build_adv_ind_pdu();

		bool connected = false;

		while (!connected) {
			int ret = adv_event();

			if (ret == 1) {
				connected = true;
			} else {
				k_msleep(ADV_INTERVAL_MS);
			}
		}

		conn_loop();

		printk("[STATE] Connection lost, returning to advertising...\n\n");
		radio_ensure_disabled();
		k_msleep(100);
	}

	return 0;
}
