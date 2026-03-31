/*
 * BLE PHY HCI Transport (HCI H4 UART) Demo
 *
 * 本 Demo 在 09_phy_ll_procedures 基础上增加了 HCI H4 传输层:
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  HCI (Host Controller Interface) 是什么?                       │
 * │                                                                 │
 * │  BLE 的分层架构中, Host 和 Controller 可以运行在不同芯片上。   │
 * │  HCI 定义了它们之间的通信规范:                                 │
 * │    - Command:  Host → Controller (如 Reset, 设置广播参数)      │
 * │    - Event:    Controller → Host (如 连接完成, 广播报告)       │
 * │    - ACL Data: 双向 (应用数据)                                 │
 * │                                                                 │
 * │  H4 是 HCI 在 UART 上最简单的封装格式:                         │
 * │    每个包前面加一个字节的 Packet Indicator:                     │
 * │    0x01 = Command, 0x02 = ACL, 0x04 = Event                   │
 * │                                                                 │
 * │  本 Demo 可以通过 Linux btattach 命令驱动:                     │
 * │    sudo btattach -B /dev/ttyUSB0 -S 115200 -P h4              │
 * │    sudo hciconfig hci0 up                                      │
 * │    hcitool lescan                                               │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ble_common.h"
#include "hal_radio.h"
#include "ll_pdu.h"
#include "adv.h"
#include "conn.h"
#include "l2cap.h"
#include "hci.h"

/*===========================================================================
 * 全局变量定义
 *===========================================================================*/

const uint8_t adv_addr[BDADDR_SIZE] = {
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

/* ★ 在 HCI 模式下, adv_data 由 Host 通过 LE_Set_Advertising_Data 设置
 * 默认提供一组初始数据, 以便无 Host 时也能广播 */
uint8_t adv_data[ADV_DATA_LEN_MAX] = {
	0x02, 0x01, 0x06,
	0x08, 0x09, 'H', 'C', 'I', '-', 'B', 'L', 'E'
};
uint8_t adv_data_len = 12;

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
struct latency_tracker lat;

struct ll_proc_conn_update proc_conn_update;
struct ll_proc_chan_map_update proc_chan_map;

volatile uint32_t adv_event_count;
volatile uint32_t conn_event_rx_ok;
volatile uint32_t conn_event_rx_timeout;
volatile uint32_t conn_event_rx_crc_err;

uint32_t last_rx_aa_us;
uint32_t last_rx_ready_us;

struct l2cap_reassembly l2cap_reasm;
struct tx_queue data_tx_q;
uint32_t l2cap_rx_count;
uint32_t l2cap_tx_count;

/* HCI 状态 */
volatile bool hci_adv_enabled;
volatile bool hci_connected;

int main(void)
{
	printk("\n");
	printk("===================================================\n");
	printk("  BLE PHY HCI Transport Demo\n");
	printk("  HCI H4 UART 传输层 + 空中接口\n");
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

	/* ★ 初始化 HCI UART 传输层 */
	h4_init();

	printk("[HCI] Waiting for Host commands on UART...\n");
	printk("[HCI] (Auto-advertise enabled as fallback)\n");

	/*
	 * ★ 主循环:
	 *
	 * 两种模式并存:
	 *   1. HCI 驱动模式: Host 通过 UART 发 HCI_LE_Set_Advertise_Enable
	 *      来启动广播, 一切由 Host 控制
	 *   2. 自动广播模式: 如果 Host 未发命令, 自动开始广播作为 fallback
	 *
	 * 无论哪种模式, 连接建立后都会:
	 *   - 通过 UART 向 Host 发送 LE Connection Complete Event
	 *   - 空中收到的数据转发为 HCI ACL Data
	 *   - Host 发来的 HCI ACL Data 在空中发出
	 */
	bool auto_adv_started = false;
	uint32_t boot_time = k_uptime_get_32();

	while (1) {
		/* 持续处理 HCI 命令 */
		h4_process();

		/*
		 * 如果 Host 在 3 秒内没有发 LE_Set_Advertise_Enable,
		 * 自动开始广播 (方便无 Host 时测试)
		 */
		bool should_adv = hci_adv_enabled;
		if (!should_adv && !auto_adv_started &&
		    (k_uptime_get_32() - boot_time) > 3000) {
			printk("[HCI] No Host detected, auto-advertising...\n");
			auto_adv_started = true;
			should_adv = true;
		}
		if (!should_adv && auto_adv_started) {
			should_adv = true;
		}

		if (!should_adv) {
			k_msleep(10);
			continue;
		}

		printk("[STATE] Entering ADVERTISING state\n");
		adv_event_count = 0;

		radio_configure_adv();
		build_adv_ind_pdu();

		bool connected = false;
		while (!connected) {
			h4_process();

			/* 如果 Host 禁用了广播, 停止 */
			if (!hci_adv_enabled && !auto_adv_started) {
				break;
			}

			int ret = adv_event();
			if (ret == 1) {
				connected = true;
			} else {
				k_msleep(ADV_INTERVAL_MS);
			}
		}

		if (connected) {
			conn_loop();

			printk("[STATE] Connection lost, returning...\n\n");
			radio_ensure_disabled();
			auto_adv_started = false;
			boot_time = k_uptime_get_32();
			k_msleep(100);
		}
	}

	return 0;
}
