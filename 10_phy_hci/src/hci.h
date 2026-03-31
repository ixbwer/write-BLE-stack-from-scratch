/*
 * BLE PHY HCI Demo — HCI H4 UART 传输层
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HCI_H
#define HCI_H

#include "ble_common.h"

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
