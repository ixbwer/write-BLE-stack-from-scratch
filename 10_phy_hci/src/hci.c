/*
 * BLE PHY HCI Demo — HCI H4 UART 传输层实现
 *
 * 本模块实现了 HCI 的 H4 UART 传输:
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  H4 传输格式:                                                   │
 * │                                                                 │
 * │  UART 字节流中, 每个 HCI 包以 1 字节 Packet Indicator 开头:    │
 * │    0x01 = HCI Command (Host → Controller)                      │
 * │    0x02 = HCI ACL Data (双向)                                  │
 * │    0x04 = HCI Event   (Controller → Host)                      │
 * │                                                                 │
 * │  本模块通过 UART 中断逐字节接收, 用状态机解析完整 HCI 包,     │
 * │  解析完成后在主循环中分发处理。                                 │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "hci.h"
#include <zephyr/drivers/uart.h>

/*===========================================================================
 * UART 设备
 *===========================================================================*/
static const struct device *uart_dev;

/*===========================================================================
 * H4 接收状态机
 *
 * 状态转换:
 *   IDLE → 收到 Indicator → TYPE_RECEIVED
 *   TYPE_RECEIVED → 收集 header → HEADER_RECEIVED
 *   HEADER_RECEIVED → 收集 payload → COMPLETE
 *   COMPLETE → 主循环处理后 → IDLE
 *===========================================================================*/
typedef enum {
	RX_IDLE,
	RX_HEADER,
	RX_PAYLOAD,
	RX_COMPLETE,
} hci_rx_state_t;

static volatile hci_rx_state_t rx_state;
static uint8_t  rx_type;           /* H4 packet indicator */
static uint8_t  rx_header[4];     /* header buffer */
static uint8_t  rx_hdr_len;       /* expected header length */
static uint8_t  rx_hdr_cnt;       /* bytes of header received */
static uint8_t  rx_payload[HCI_ACL_MAX_DATA];
static uint16_t rx_pay_len;       /* expected payload length */
static uint16_t rx_pay_cnt;       /* bytes of payload received */

/* ACL data from Host (pending TX to air) */
static uint8_t  acl_tx_buf[HCI_ACL_MAX_DATA];
static uint16_t acl_tx_len;
static volatile bool acl_tx_ready;

/*===========================================================================
 * UART TX: 逐字节 polling 发送 (简单可靠)
 *===========================================================================*/
static void uart_tx_byte(uint8_t byte)
{
	uart_poll_out(uart_dev, byte);
}

static void uart_tx_buf(const uint8_t *buf, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++) {
		uart_tx_byte(buf[i]);
	}
}

/*===========================================================================
 * HCI Event 发送辅助函数
 *===========================================================================*/

/*
 * 发送 Command Complete Event
 *
 * [0x04] [0x0E] [length] [num_cmd=1] [opcode_lo] [opcode_hi] [params...]
 */
static void send_cmd_complete(uint16_t opcode, const uint8_t *params,
			      uint8_t params_len)
{
	uint8_t total_len = 3 + params_len; /* num_cmd(1) + opcode(2) + params */

	uart_tx_byte(HCI_H4_EVT);
	uart_tx_byte(HCI_EVT_CMD_COMPLETE);
	uart_tx_byte(total_len);
	uart_tx_byte(0x01);                  /* Num_HCI_Command_Packets */
	uart_tx_byte(opcode & 0xFF);         /* OpCode low */
	uart_tx_byte((opcode >> 8) & 0xFF);  /* OpCode high */
	uart_tx_buf(params, params_len);
}

/* 发送 Status=Success 的 Command Complete */
static void send_cmd_complete_status(uint16_t opcode)
{
	uint8_t status = 0x00;
	send_cmd_complete(opcode, &status, 1);
}

/*===========================================================================
 * HCI Command 处理分发
 *
 * 处理来自 Host 的 HCI Command, 返回对应的 Event。
 * 本 Demo 实现 BlueZ 初始化所需的最小命令集。
 *===========================================================================*/
static void handle_hci_cmd(void)
{
	uint16_t opcode = (uint16_t)rx_header[0] | ((uint16_t)rx_header[1] << 8);

	printk("[HCI] CMD opcode=0x%04X len=%u\n", opcode, rx_pay_len);

	switch (opcode) {

	case HCI_OP_RESET:
		/* 复位内部状态 */
		hci_adv_enabled = false;
		printk("[HCI] → Reset\n");
		send_cmd_complete_status(opcode);
		break;

	case HCI_OP_READ_LOCAL_VERSION: {
		/* HCI Version: BLE 5.0 = 0x09
		 * HCI Revision: 0x0000
		 * LMP Version: 9
		 * Manufacturer: 0xFFFF (DIY)
		 * LMP Subversion: 0x0001
		 */
		uint8_t rsp[] = {
			0x00,                   /* Status */
			0x09,                   /* HCI_Version (BLE 5.0) */
			0x00, 0x00,             /* HCI_Revision */
			0x09,                   /* LMP_Version */
			0xFF, 0xFF,             /* Manufacturer_Name */
			0x01, 0x00,             /* LMP_Subversion */
		};
		printk("[HCI] → Read Local Version\n");
		send_cmd_complete(opcode, rsp, sizeof(rsp));
		break;
	}

	case HCI_OP_READ_LOCAL_COMMANDS: {
		/* 返回支持的命令位图 (256 bit = 64 字节)
		 * 简单起见: 标记我们实际支持的几个命令 */
		uint8_t rsp[65];
		memset(rsp, 0, sizeof(rsp));
		rsp[0] = 0x00; /* Status */
		/* Byte 5, bit 6: HCI_Reset */
		rsp[1 + 5] |= BIT(6);
		/* Byte 14, bit 3: Read_Local_Version */
		rsp[1 + 14] |= BIT(3);
		/* Byte 25, bit 0: LE_Set_Event_Mask */
		rsp[1 + 25] |= BIT(0);
		/* Byte 25, bit 1: LE_Read_Buffer_Size */
		rsp[1 + 25] |= BIT(1);
		/* Byte 25, bit 2: LE_Read_Local_Features */
		rsp[1 + 25] |= BIT(2);
		/* Byte 25, bit 5: LE_Set_Adv_Param */
		rsp[1 + 25] |= BIT(5);
		/* Byte 25, bit 7: LE_Set_Adv_Data */
		rsp[1 + 25] |= BIT(7);
		/* Byte 26, bit 1: LE_Set_Adv_Enable */
		rsp[1 + 26] |= BIT(1);
		printk("[HCI] → Read Local Commands\n");
		send_cmd_complete(opcode, rsp, sizeof(rsp));
		break;
	}

	case HCI_OP_READ_LOCAL_FEATURES: {
		/* LMP Features - mark LE Supported */
		uint8_t rsp[9];
		memset(rsp, 0, sizeof(rsp));
		rsp[0] = 0x00; /* Status */
		rsp[1 + 4] = BIT(6); /* Byte 4 bit 6: LE Supported (Controller) */
		printk("[HCI] → Read Local Features\n");
		send_cmd_complete(opcode, rsp, sizeof(rsp));
		break;
	}

	case HCI_OP_READ_BD_ADDR: {
		/* 返回我们的广播地址 */
		uint8_t rsp[7];
		rsp[0] = 0x00; /* Status */
		memcpy(&rsp[1], adv_addr, BDADDR_SIZE);
		printk("[HCI] → Read BD ADDR\n");
		send_cmd_complete(opcode, rsp, sizeof(rsp));
		break;
	}

	case HCI_OP_SET_EVENT_MASK:
		/* 存储事件掩码 (本 Demo 忽略, 直接成功) */
		printk("[HCI] → Set Event Mask\n");
		send_cmd_complete_status(opcode);
		break;

	case HCI_OP_LE_SET_EVENT_MASK:
		printk("[HCI] → LE Set Event Mask\n");
		send_cmd_complete_status(opcode);
		break;

	case HCI_OP_LE_READ_BUFFER_SIZE: {
		/* 报告 Controller 的 LE ACL 缓冲能力 */
		uint8_t rsp[] = {
			0x00,                   /* Status */
			LL_DATA_MTU_DEFAULT,    /* LE_ACL_Data_Packet_Length low */
			0x00,                   /* LE_ACL_Data_Packet_Length high */
			TX_QUEUE_SIZE,          /* Total_Num_LE_ACL_Data_Packets */
		};
		printk("[HCI] → LE Read Buffer Size\n");
		send_cmd_complete(opcode, rsp, sizeof(rsp));
		break;
	}

	case HCI_OP_LE_READ_LOCAL_FEATURES: {
		/* LE Features: 全零 (不支持任何可选特性) */
		uint8_t rsp[9];
		memset(rsp, 0, sizeof(rsp));
		rsp[0] = 0x00;
		printk("[HCI] → LE Read Local Features\n");
		send_cmd_complete(opcode, rsp, sizeof(rsp));
		break;
	}

	case HCI_OP_LE_SET_ADV_PARAM:
		/* 简化: 忽略参数, 直接成功 */
		printk("[HCI] → LE Set Adv Params\n");
		send_cmd_complete_status(opcode);
		break;

	case HCI_OP_LE_READ_ADV_TX_POWER: {
		uint8_t rsp[] = {
			0x00,  /* Status */
			0x00,  /* TX Power Level = 0 dBm */
		};
		printk("[HCI] → LE Read Adv TX Power\n");
		send_cmd_complete(opcode, rsp, sizeof(rsp));
		break;
	}

	case HCI_OP_LE_SET_ADV_DATA:
		/* 参数: [Length(1)] [Data(31)] */
		if (rx_pay_len >= 1) {
			uint8_t data_len = rx_payload[0];
			if (data_len > ADV_DATA_LEN_MAX) {
				data_len = ADV_DATA_LEN_MAX;
			}
			memcpy(adv_data, &rx_payload[1], data_len);
			adv_data_len = data_len;
			printk("[HCI] → LE Set Adv Data (len=%u)\n", data_len);
		}
		send_cmd_complete_status(opcode);
		break;

	case HCI_OP_LE_SET_ADV_ENABLE:
		/* 参数: [Enable(1)]: 0x00=disable, 0x01=enable */
		if (rx_pay_len >= 1 && rx_payload[0] == 0x01) {
			hci_adv_enabled = true;
			printk("[HCI] → Advertising ENABLED\n");
		} else {
			hci_adv_enabled = false;
			printk("[HCI] → Advertising DISABLED\n");
		}
		send_cmd_complete_status(opcode);
		break;

	case HCI_OP_LE_READ_WHITE_LIST_SIZE: {
		uint8_t rsp[] = {0x00, 0x00}; /* Status, Size=0 */
		send_cmd_complete(opcode, rsp, sizeof(rsp));
		break;
	}

	case HCI_OP_LE_READ_SUPP_STATES: {
		/* LE Supported States: 8 bytes bitmask */
		uint8_t rsp[9];
		memset(rsp, 0, sizeof(rsp));
		rsp[0] = 0x00; /* Status */
		/* Bit 0: Non-connectable Adv */
		/* Bit 1: Scannable Adv */
		/* Bit 2: Connectable Adv */
		/* Bit 8: Passive scanning */
		rsp[1] = 0x07; /* lowest 3 bits: adv states */
		printk("[HCI] → LE Read Supported States\n");
		send_cmd_complete(opcode, rsp, sizeof(rsp));
		break;
	}

	default:
		/* 未知命令 → Command Complete with error 0x01 (Unknown Command) */
		printk("[HCI] → Unknown cmd 0x%04X\n", opcode);
		{
			uint8_t status = 0x01; /* Unknown HCI Command */
			send_cmd_complete(opcode, &status, 1);
		}
		break;
	}
}

/*===========================================================================
 * HCI ACL Data 处理 (Host → Controller)
 *
 * H4 ACL:
 *   [0x02] [Handle_Lo] [Handle_Hi|PB|BC] [Length_Lo] [Length_Hi] [Data...]
 *
 * Handle: 12 bit (我们只支持 handle 0x0000)
 * PB Flag: bits 13-12 (00=first, 01=continuation)
 * BC Flag: bits 15-14 (00 for BLE)
 *===========================================================================*/
static void handle_hci_acl(void)
{
	uint16_t handle_flags = (uint16_t)rx_header[0] |
				((uint16_t)rx_header[1] << 8);
	uint16_t handle = handle_flags & 0x0FFF;
	uint16_t data_len = (uint16_t)rx_header[2] |
			    ((uint16_t)rx_header[3] << 8);

	printk("[HCI] ACL RX handle=0x%03X len=%u\n", handle, data_len);

	if (handle != HCI_CONN_HANDLE || !hci_connected) {
		printk("[HCI] ACL: invalid handle or not connected\n");
		return;
	}

	if (data_len > 0 && data_len <= HCI_ACL_MAX_DATA && !acl_tx_ready) {
		memcpy(acl_tx_buf, rx_payload, data_len);
		acl_tx_len = data_len;
		acl_tx_ready = true;
	}
}

/*===========================================================================
 * UART ISR — H4 逐字节状态机
 *
 * 这是 HCI 传输层的核心: 在中断中逐字节接收,
 * 根据 H4 Packet Indicator 判断包类型, 收集 header + payload,
 * 收完后标记 RX_COMPLETE, 由主循环处理。
 *===========================================================================*/
static void uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (!uart_irq_update(dev)) {
		return;
	}

	while (uart_irq_rx_ready(dev)) {
		uint8_t byte;
		int ret = uart_fifo_read(dev, &byte, 1);
		if (ret != 1) {
			break;
		}

		switch (rx_state) {
		case RX_IDLE:
			if (byte == HCI_H4_CMD) {
				rx_type = HCI_H4_CMD;
				rx_hdr_len = 3; /* OpCode(2) + ParamLen(1) */
				rx_hdr_cnt = 0;
				rx_state = RX_HEADER;
			} else if (byte == HCI_H4_ACL) {
				rx_type = HCI_H4_ACL;
				rx_hdr_len = 4; /* Handle(2) + DataLen(2) */
				rx_hdr_cnt = 0;
				rx_state = RX_HEADER;
			}
			/* 忽略其他字节 (包括 0x04 Event 和垃圾字节) */
			break;

		case RX_HEADER:
			rx_header[rx_hdr_cnt++] = byte;
			if (rx_hdr_cnt >= rx_hdr_len) {
				if (rx_type == HCI_H4_CMD) {
					rx_pay_len = rx_header[2];
				} else { /* ACL */
					rx_pay_len = (uint16_t)rx_header[2] |
						     ((uint16_t)rx_header[3] << 8);
				}
				rx_pay_cnt = 0;

				if (rx_pay_len == 0) {
					rx_state = RX_COMPLETE;
				} else if (rx_pay_len > HCI_ACL_MAX_DATA) {
					/* 过长, 丢弃 */
					rx_state = RX_IDLE;
				} else {
					rx_state = RX_PAYLOAD;
				}
			}
			break;

		case RX_PAYLOAD:
			if (rx_pay_cnt < HCI_ACL_MAX_DATA) {
				rx_payload[rx_pay_cnt] = byte;
			}
			rx_pay_cnt++;
			if (rx_pay_cnt >= rx_pay_len) {
				rx_state = RX_COMPLETE;
			}
			break;

		case RX_COMPLETE:
			/* 主循环还没处理上一个包, 丢弃新数据 */
			break;
		}
	}
}

/*===========================================================================
 * 公共接口
 *===========================================================================*/

void h4_init(void)
{
	uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	if (!device_is_ready(uart_dev)) {
		printk("[HCI] UART device not ready!\n");
		return;
	}

	rx_state = RX_IDLE;
	acl_tx_ready = false;

	uart_irq_callback_set(uart_dev, uart_isr);
	uart_irq_rx_enable(uart_dev);

	printk("[HCI] H4 UART initialized\n");
}

void h4_process(void)
{
	if (rx_state != RX_COMPLETE) {
		return;
	}

	if (rx_type == HCI_H4_CMD) {
		handle_hci_cmd();
	} else if (rx_type == HCI_H4_ACL) {
		handle_hci_acl();
	}

	rx_state = RX_IDLE;
}

void hci_send_le_conn_complete(void)
{
	/*
	 * LE Connection Complete Event (LE Meta Event):
	 * [0x04] [0x3E] [Length] [Subevent=0x01] [Status=0]
	 * [Handle_Lo] [Handle_Hi] [Role=Slave(0x01)]
	 * [Peer_Addr_Type] [Peer_Addr(6)]
	 * [Conn_Interval(2)] [Conn_Latency(2)] [Supervision_Timeout(2)]
	 * [Master_Clock_Accuracy]
	 */
	uint8_t evt[19];
	evt[0]  = 0x01;                                 /* Subevent */
	evt[1]  = 0x00;                                 /* Status: Success */
	evt[2]  = HCI_CONN_HANDLE & 0xFF;               /* Handle low */
	evt[3]  = (HCI_CONN_HANDLE >> 8) & 0xFF;        /* Handle high */
	evt[4]  = 0x01;                                 /* Role: Slave */
	evt[5]  = 0x01;                                 /* Peer Addr Type: Random */
	memcpy(&evt[6], conn_params.peer_addr, BDADDR_SIZE); /* Peer Addr */
	evt[12] = conn_params.interval & 0xFF;
	evt[13] = (conn_params.interval >> 8) & 0xFF;
	evt[14] = conn_params.latency & 0xFF;
	evt[15] = (conn_params.latency >> 8) & 0xFF;
	evt[16] = conn_params.timeout & 0xFF;
	evt[17] = (conn_params.timeout >> 8) & 0xFF;
	evt[18] = conn_params.sca;                      /* Master Clock Accuracy */

	uart_tx_byte(HCI_H4_EVT);
	uart_tx_byte(HCI_EVT_LE_META);
	uart_tx_byte(sizeof(evt));
	uart_tx_buf(evt, sizeof(evt));

	printk("[HCI] TX: LE Connection Complete Event\n");
}

void hci_send_disconnect_complete(uint8_t reason)
{
	/*
	 * Disconnection Complete Event:
	 * [0x04] [0x05] [Length=4] [Status=0]
	 * [Handle_Lo] [Handle_Hi] [Reason]
	 */
	uart_tx_byte(HCI_H4_EVT);
	uart_tx_byte(HCI_EVT_DISCONNECT_COMPLETE);
	uart_tx_byte(0x04);
	uart_tx_byte(0x00);                     /* Status: Success */
	uart_tx_byte(HCI_CONN_HANDLE & 0xFF);
	uart_tx_byte((HCI_CONN_HANDLE >> 8) & 0xFF);
	uart_tx_byte(reason);

	printk("[HCI] TX: Disconnect Complete (reason=0x%02X)\n", reason);
}

void hci_send_num_completed_pkts(uint16_t num)
{
	/*
	 * Number of Completed Packets Event:
	 * [0x04] [0x13] [Length] [Num_Handles=1]
	 * [Handle_Lo] [Handle_Hi] [Num_Completed_Lo] [Num_Completed_Hi]
	 */
	uart_tx_byte(HCI_H4_EVT);
	uart_tx_byte(HCI_EVT_NUM_COMPLETED_PKTS);
	uart_tx_byte(5);               /* Length */
	uart_tx_byte(1);               /* Num_Handles */
	uart_tx_byte(HCI_CONN_HANDLE & 0xFF);
	uart_tx_byte((HCI_CONN_HANDLE >> 8) & 0xFF);
	uart_tx_byte(num & 0xFF);
	uart_tx_byte((num >> 8) & 0xFF);
}

void hci_send_acl_data(const uint8_t *data, uint16_t len,
		       bool is_first_fragment)
{
	/*
	 * HCI ACL Data (Controller → Host):
	 * [0x02] [Handle_Lo] [Handle_Hi|PB|BC] [DataLen_Lo] [DataLen_Hi] [Data...]
	 * PB: 00=first automatically flushable, 01=continuing
	 */
	uint16_t handle_flags = HCI_CONN_HANDLE;
	if (!is_first_fragment) {
		handle_flags |= (0x01 << 12); /* PB = continuing */
	} else {
		handle_flags |= (0x02 << 12); /* PB = first automatically flushable */
	}

	uart_tx_byte(HCI_H4_ACL);
	uart_tx_byte(handle_flags & 0xFF);
	uart_tx_byte((handle_flags >> 8) & 0xFF);
	uart_tx_byte(len & 0xFF);
	uart_tx_byte((len >> 8) & 0xFF);
	uart_tx_buf(data, len);
}

bool hci_acl_tx_pending(void)
{
	return acl_tx_ready;
}

uint16_t hci_acl_tx_get(uint8_t *buf, uint16_t buf_size)
{
	if (!acl_tx_ready) {
		return 0;
	}
	uint16_t copy_len = acl_tx_len;
	if (copy_len > buf_size) {
		copy_len = buf_size;
	}
	memcpy(buf, acl_tx_buf, copy_len);
	acl_tx_ready = false;
	return copy_len;
}
