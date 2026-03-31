/*
 * BLE PHY CONNECT_IND 解析 Demo
 *
 * 本 Demo 专注于 BLE 连接建立的第一步 —— 接收和解析 CONNECT_IND。
 *
 * ┌ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┐
 * │ 什么是 CONNECT_IND?                                                   │
 * │                                                                       │
 * │ CONNECT_IND 是 Master 发给 Slave 的连接请求包, 包含建立连接所需的     │
 * │ 全部参数: Access Address, CRC 初始值, 连接间隔, 跳频参数等。          │
 * │                                                                       │
 * │ Core Spec Vol 6, Part B, Section 2.3.3.1:                             │
 * │   InitA(6) + AdvA(6) + LLData(22) = 34 字节                          │
 * │                                                                       │
 * │ LLData 包含 12 个字段:                                                │
 * │   AA, CRCInit, WinSize, WinOffset, Interval, Latency, Timeout,       │
 * │   ChM, Hop, SCA                                                       │
 * └ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┘
 *
 * 时序:
 *
 *   Slave                               Master
 *    │                                    │
 *    │──── ADV_IND (ch 37/38/39) ───────>│
 *    │                                    │
 *    │<──── CONNECT_IND (T_IFS=150μs) ───│
 *    │        [T0 = RTC0 timestamp]       │
 *    │                                    │
 *    │  解析 LLData → print_connect_ind() │
 *    │                                    │
 *    │  (本例程不进入连接态, 打印后回到广播)│
 *
 * 代码结构:
 *   ble_common.h  — 公共类型、常量 (新增 struct conn_param)
 *   hal_radio.c/h — HAL 层 (新增 rtc0_start)
 *   ll_pdu.c/h    — PDU 构造 (新增 CONNECT_IND 解析与打印)
 *   adv.c/h       — 广播态 (改为等待 CONNECT_IND, 不再处理 SCAN_REQ)
 *   main.c        — 主状态机: ADV → CONNECT_IND → 打印 → ADV
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ble_common.h"
#include "hal_radio.h"
#include "ll_pdu.h"
#include "adv.h"

/*===========================================================================
 * 全局变量定义
 *===========================================================================*/

const uint8_t adv_addr[BDADDR_SIZE] = {
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

const uint8_t adv_data[] = {
	0x02, 0x01, 0x06,
	0x08, 0x09, 'C', 'o', 'n', 'n', 'I', 'n', 'd'
};

struct conn_param conn_params;

/* PDU 缓冲区 */
struct pdu_adv pdu_adv_ind __aligned(4);
struct pdu_adv pdu_adv_rx __aligned(4);

/* CONNECT_IND 接收时刻 (NRF_RTC0 Counter) — 这就是 "T0" */
uint32_t connect_end_rtc;

/* SCA PPM 映射表 (BLE Core Spec Vol 6, Part B, 4.5.7, Table 4.1) */
const uint16_t sca_ppm_table[] = {500, 250, 150, 100, 75, 50, 30, 20};

/* 统计 */
volatile uint32_t adv_event_count;

/*===========================================================================
 * 主函数
 *===========================================================================*/
int main(void)
{
	printk("\n");
	printk("===================================================\n");
	printk("  BLE PHY CONNECT_IND Demo\n");
	printk("  解析连接请求中的 12 个参数字段\n");
	printk("===================================================\n");
	printk("Device: %s  Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
	       ADV_NAME,
	       adv_addr[5], adv_addr[4], adv_addr[3],
	       adv_addr[2], adv_addr[1], adv_addr[0]);
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

		/* ★ 收到 CONNECT_IND! 打印解析结果 */
		print_connect_ind();

		printk("\n[STATE] CONNECT_IND parsed. "
		       "(Not entering connection state in this demo)\n");
		printk("[STATE] Returning to advertising...\n\n");

		radio_ensure_disabled();
		k_msleep(100);
	}

	return 0;
}
