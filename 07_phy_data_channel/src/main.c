/*
 * BLE PHY Data Channel (数据信道) Demo
 *
 * 本 Demo 在 06_phy_anchor 的基础上增加了 L2CAP 数据通道功能:
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  新增功能                                                       │
 * │                                                                 │
 * │  1. LL Data PDU 收发: 区分 LLID=Start/Continue/Control         │
 * │  2. L2CAP 帧重组: 将多个分片合并为完整的 L2CAP Basic Frame     │
 * │  3. L2CAP 帧分片: 将完整帧拆分为 ≤27 字节的 PDU 分片          │
 * │  4. Echo 回环: 收到的 L2CAP 数据原样发回, 验证数据通道工作     │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * L2CAP Basic Frame 格式:
 *
 *   字节偏移  字段           说明
 *   [0..1]    Length (LE)    Payload 长度 (不含此 4 字节 header)
 *   [2..3]    Channel ID     0x0004=ATT, 0x0005=L2CAP Signaling, 0x0006=SMP
 *   [4..]     Payload        上层数据 (ATT/SMP/...)
 *
 * 空口分片过程 (以 40 字节 ATT 数据为例):
 *
 *   L2CAP Frame: [Length=40] [CID=0x0004] [40 bytes ATT data]
 *   Total = 44 bytes
 *
 *   LL PDU #1: LLID=Start,     len=27, payload=[L2CAP hdr + 前 23 bytes]
 *   LL PDU #2: LLID=Continue,  len=17, payload=[后 17 bytes]
 *
 * 代码结构 (在 03 基础上新增 l2cap 模块):
 *   ble_common.h  — 公共类型 (新增 L2CAP 常量、重组缓冲区、TX 队列)
 *   hal_radio.c/h — HAL 层 (与 03 相同)
 *   ll_pdu.c/h    — 链路层 PDU (新增 LL_LENGTH_RSP)
 *   adv.c/h       — 广播态 (与 03 相同)
 *   conn.c/h      — 连接态 (新增: 数据 PDU 处理、L2CAP 分片发送)
 *   l2cap.c/h     — ★ 新增: L2CAP 帧重组、分片、回环处理
 *   main.c        — 全局变量定义与主状态机
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ble_common.h"
#include "hal_radio.h"
#include "ll_pdu.h"
#include "adv.h"
#include "conn.h"
#include "l2cap.h"

/*===========================================================================
 * 全局变量定义
 *===========================================================================*/

const uint8_t adv_addr[BDADDR_SIZE] = {
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

const uint8_t adv_data[] = {
	0x02, 0x01, 0x06,
	0x08, 0x09, 'D', 'a', 't', 'a', 'C', 'h', 'a'
};

struct conn_param conn_params;

uint8_t tx_sn;
uint8_t rx_nesn;
uint16_t conn_event_counter;
uint8_t last_unmapped_chan;
volatile bool conn_terminated;

struct pdu_data tx_pdu_buf __aligned(4);
bool tx_pdu_pending;

struct pdu_adv pdu_adv_ind __aligned(4);
struct pdu_adv pdu_adv_rx __aligned(4);
struct pdu_data pdu_data_rx __aligned(4);

uint32_t connect_end_rtc;

const uint16_t sca_ppm_table[] = {500, 250, 150, 100, 75, 50, 30, 20};

struct anchor_tracker anchor;

volatile uint32_t adv_event_count;
volatile uint32_t conn_event_rx_ok;
volatile uint32_t conn_event_rx_timeout;
volatile uint32_t conn_event_rx_crc_err;

uint32_t last_rx_aa_us;
uint32_t last_rx_ready_us;

/* L2CAP 重组缓冲区 */
struct l2cap_reassembly l2cap_reasm;

/* 数据 TX 队列 */
struct tx_queue data_tx_q;

/* 数据统计 */
uint32_t l2cap_rx_count;
uint32_t l2cap_tx_count;

/* L2CAP 延迟日志 */
uint16_t l2cap_last_rx_cid;
uint16_t l2cap_last_rx_len;
int      l2cap_last_tx_frags;
bool     l2cap_last_rx_pending;

/*===========================================================================
 * 主函数
 *===========================================================================*/
int main(void)
{
	printk("\n");
	printk("===================================================\n");
	printk("  BLE PHY Data Channel Demo\n");
	printk("  演示 L2CAP 数据帧的分片、重组与回环\n");
	printk("===================================================\n");
	printk("Device: %s  Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
	       ADV_NAME,
	       adv_addr[5], adv_addr[4], adv_addr[3],
	       adv_addr[2], adv_addr[1], adv_addr[0]);
	printk("\n");
	printk("数据通道功能:\n");
	printk("  1. 接收 LL Data PDU (LLID=Start/Continue)\n");
	printk("  2. 重组为完整 L2CAP Basic Frame\n");
	printk("  3. 拆分为 <=27 字节的 PDU 分片发回 (Echo)\n");
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
