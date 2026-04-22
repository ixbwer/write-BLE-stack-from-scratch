/*
 * BLE PHY HCI Demo — HCI H4 UART 传输层
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HCI_H
#define HCI_H

#include "ble_common.h"

/*===========================================================================
 * HCI 专属常量
 *===========================================================================*/

/* H4 Packet Indicators */
#ifndef HCI_H4_CMD
#define HCI_H4_CMD          0x01
#endif

#ifndef HCI_H4_ACL
#define HCI_H4_ACL          0x02
#endif

#ifndef HCI_H4_EVT
#define HCI_H4_EVT          0x04
#endif

/* HCI Event Codes */
#ifndef HCI_EVT_DISCONNECT_COMPLETE
#define HCI_EVT_DISCONNECT_COMPLETE  0x05
#endif

#ifndef HCI_EVT_CMD_COMPLETE
#define HCI_EVT_CMD_COMPLETE         0x0E
#endif

#ifndef HCI_EVT_CMD_STATUS
#define HCI_EVT_CMD_STATUS           0x0F
#endif

#ifndef HCI_EVT_NUM_COMPLETED_PKTS
#define HCI_EVT_NUM_COMPLETED_PKTS   0x13
#endif

#ifndef HCI_EVT_LE_META
#define HCI_EVT_LE_META              0x3E
#endif

/* LE Meta Subevent Codes */
#ifndef HCI_LE_SUBEVENT_CONN_COMPLETE
#define HCI_LE_SUBEVENT_CONN_COMPLETE  0x01
#endif

/* HCI Command OpCodes (OGF | OCF) */
#ifndef HCI_OP_RESET
#define HCI_OP_RESET                    0x0C03
#endif

#ifndef HCI_OP_SET_EVENT_MASK
#define HCI_OP_SET_EVENT_MASK           0x0C01
#endif

#ifndef HCI_OP_READ_LOCAL_VERSION
#define HCI_OP_READ_LOCAL_VERSION       0x1001
#endif

#ifndef HCI_OP_READ_LOCAL_COMMANDS
#define HCI_OP_READ_LOCAL_COMMANDS      0x1002
#endif

#ifndef HCI_OP_READ_LOCAL_FEATURES
#define HCI_OP_READ_LOCAL_FEATURES      0x1003
#endif

#ifndef HCI_OP_READ_BD_ADDR
#define HCI_OP_READ_BD_ADDR             0x1009
#endif

#ifndef HCI_OP_LE_SET_EVENT_MASK
#define HCI_OP_LE_SET_EVENT_MASK        0x2001
#endif

#ifndef HCI_OP_LE_READ_BUFFER_SIZE
#define HCI_OP_LE_READ_BUFFER_SIZE      0x2002
#endif

#ifndef HCI_OP_LE_READ_LOCAL_FEATURES
#define HCI_OP_LE_READ_LOCAL_FEATURES   0x2003
#endif

#ifndef HCI_OP_LE_SET_ADV_PARAM
#define HCI_OP_LE_SET_ADV_PARAM         0x2006
#endif

#ifndef HCI_OP_LE_READ_ADV_TX_POWER
#define HCI_OP_LE_READ_ADV_TX_POWER     0x2007
#endif

#ifndef HCI_OP_LE_SET_ADV_DATA
#define HCI_OP_LE_SET_ADV_DATA          0x2008
#endif

#ifndef HCI_OP_LE_SET_ADV_ENABLE
#define HCI_OP_LE_SET_ADV_ENABLE        0x200A
#endif

#ifndef HCI_OP_LE_SET_SCAN_PARAM
#define HCI_OP_LE_SET_SCAN_PARAM        0x200B
#endif

#ifndef HCI_OP_LE_SET_SCAN_ENABLE
#define HCI_OP_LE_SET_SCAN_ENABLE       0x200C
#endif

#ifndef HCI_OP_LE_READ_WHITE_LIST_SIZE
#define HCI_OP_LE_READ_WHITE_LIST_SIZE  0x200F
#endif

#ifndef HCI_OP_LE_READ_SUPP_STATES
#define HCI_OP_LE_READ_SUPP_STATES      0x201C
#endif

/* ACL */
#ifndef HCI_ACL_HDR_SIZE
#define HCI_ACL_HDR_SIZE   4
#endif

#ifndef HCI_ACL_MAX_DATA
#define HCI_ACL_MAX_DATA   256
#endif

/* 连接句柄 (本 Demo 只支持 1 条连接, 固定句柄 0x0000) */
#ifndef HCI_CONN_HANDLE
#define HCI_CONN_HANDLE   0x0000
#endif

/* HCI 状态 */
extern volatile bool hci_adv_enabled;
extern volatile bool hci_connected;

/* 初始化 HCI UART (中断接收模式) */
void h4_init(void);

/* 在主循环中调用: 处理已接收完整的 HCI 包 */
void h4_process(void);

/* 发送 HCI LE Connection Complete Event */
void hci_send_le_conn_complete(void);

/* 发送 HCI Disconnection Complete Event */
void hci_send_disconnect_complete(uint8_t reason);

/* 发送 HCI Number of Completed Packets Event */
void hci_send_num_completed_pkts(uint16_t num);

/* 将空中收到的 L2CAP 数据作为 HCI ACL Data 上报给 Host */
void hci_send_acl_data(const uint8_t *data, uint16_t len,
		       bool is_first_fragment);

/* 检查 Host 是否发来了 ACL 数据待发送 */
bool hci_acl_tx_pending(void);

/* 获取 Host 发来的 ACL 数据 (返回长度, 0=无数据) */
uint16_t hci_acl_tx_get(uint8_t *buf, uint16_t buf_size);

#endif /* HCI_H */
