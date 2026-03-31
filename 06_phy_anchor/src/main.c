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
 * 代码结构:
 *   ble_common.h  — 公共类型、常量与全局变量声明
 *   hal_radio.c/h — HAL 层: 硬件初始化与 Radio 配置
 *   ll_pdu.c/h    — 链路层: PDU 构造、LL Control、信道选择
 *   adv.c/h       — 广播态: ADV_IND 发送与 CONNECT_IND 接收
 *   conn.c/h      — 连接态: 连接事件与锚点调度
 *   main.c        — 全局变量定义与主状态机
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ble_common.h"
#include "hal_radio.h"
#include "ll_pdu.h"
#include "adv.h"
#include "conn.h"

/*===========================================================================
 * 全局变量定义
 *===========================================================================*/

const uint8_t adv_addr[BDADDR_SIZE] = {
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

const uint8_t adv_data[] = {
	0x02, 0x01, 0x06,
	0x07, 0x09, 'A', 'n', 'c', 'h', 'o', 'r'
};

struct conn_param conn_params;

/* 连接态变量 */
uint8_t tx_sn;
uint8_t rx_nesn;
uint16_t conn_event_counter;
uint8_t last_unmapped_chan;
volatile bool conn_terminated;

struct pdu_data tx_pdu_buf __aligned(4);
bool tx_pdu_pending;

/* PDU 缓冲区 */
struct pdu_adv pdu_adv_ind __aligned(4);
struct pdu_adv pdu_adv_rx __aligned(4);
struct pdu_data pdu_data_rx __aligned(4);

/* CONNECT_IND 接收时刻 (NRF_RTC0 Counter) — 这就是 "T0", 锚点的起始基准 */
uint32_t connect_end_rtc;

/* SCA PPM 映射表 */
const uint16_t sca_ppm_table[] = {500, 250, 150, 100, 75, 50, 30, 20};

/* 锚点追踪 */
struct anchor_tracker anchor;

/* 统计 */
volatile uint32_t adv_event_count;
volatile uint32_t conn_event_rx_ok;
volatile uint32_t conn_event_rx_timeout;
volatile uint32_t conn_event_rx_crc_err;

/* Drift correction: TIMER0 captures from last successful RX */
uint32_t last_rx_aa_us;
uint32_t last_rx_ready_us;

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
