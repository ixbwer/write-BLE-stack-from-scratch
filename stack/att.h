/*
 * BLE PHY ATT Demo — Attribute Protocol Server
 *
 * 最小 ATT 实现, 支持:
 *   - Exchange MTU (0x02/0x03)
 *   - Find Information (0x04/0x05)
 *   - Read By Type (0x08/0x09)
 *   - Read (0x0A/0x0B)
 *   - Read By Group Type (0x10/0x11)
 *   - Write (0x12/0x13)
 *   - Error Response (0x01)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ATT_H
#define ATT_H

#include "ble_common.h"

/* ATT Opcodes */
#define ATT_OP_ERROR_RSP           0x01
#define ATT_OP_MTU_REQ             0x02
#define ATT_OP_MTU_RSP             0x03
#define ATT_OP_FIND_INFO_REQ       0x04
#define ATT_OP_FIND_INFO_RSP       0x05
#define ATT_OP_FIND_TYPE_REQ       0x06
#define ATT_OP_FIND_TYPE_RSP       0x07
#define ATT_OP_READ_TYPE_REQ       0x08
#define ATT_OP_READ_TYPE_RSP       0x09
#define ATT_OP_READ_REQ            0x0A
#define ATT_OP_READ_RSP            0x0B
#define ATT_OP_READ_GROUP_REQ      0x10
#define ATT_OP_READ_GROUP_RSP      0x11
#define ATT_OP_WRITE_REQ           0x12
#define ATT_OP_WRITE_RSP           0x13
#define ATT_OP_WRITE_CMD           0x52
#define ATT_OP_NOTIFY              0x1B
#define ATT_OP_INDICATE            0x1D
#define ATT_OP_CONFIRM             0x1E

/* ATT Error Codes */
#define ATT_ERR_INVALID_HANDLE         0x01
#define ATT_ERR_READ_NOT_PERMITTED     0x02
#define ATT_ERR_WRITE_NOT_PERMITTED    0x03
#define ATT_ERR_INVALID_PDU            0x04
#define ATT_ERR_NOT_SUPPORTED          0x06
#define ATT_ERR_INVALID_OFFSET         0x07
#define ATT_ERR_ATTR_NOT_FOUND         0x0A
#define ATT_ERR_UNSUPPORTED_GROUP_TYPE 0x10

/* ATT Default MTU */
#define ATT_MTU_DEFAULT  23

/* Well-known GATT UUIDs (16-bit) */
#define UUID_PRIMARY_SERVICE    0x2800
#define UUID_SECONDARY_SERVICE  0x2801
#define UUID_CHARACTERISTIC     0x2803
#define UUID_CCC_DESCRIPTOR     0x2902

/* Well-known GAP/GATT Service UUIDs */
#define UUID_GAP_SERVICE        0x1800
#define UUID_GATT_SERVICE       0x1801
#define UUID_DEVICE_NAME        0x2A00
#define UUID_APPEARANCE         0x2A01

/* Characteristic Properties */
#define CHRC_PROP_READ          0x02
#define CHRC_PROP_WRITE_NO_RSP  0x04
#define CHRC_PROP_WRITE         0x08
#define CHRC_PROP_NOTIFY        0x10
#define CHRC_PROP_INDICATE      0x20

/* Attribute Permissions */
#define ATT_PERM_READ           0x01
#define ATT_PERM_WRITE          0x02

/*
 * 处理 CID=0x0004 的 ATT 帧
 */
void att_handle(const uint8_t *payload, uint16_t len);

/*
 * 初始化 ATT 状态
 */
void att_init(void);

/*
 * 发送 Handle Value Notification (Server → Client, 无需确认)
 */
void att_notify(uint16_t handle, const uint8_t *value, uint16_t len);

#endif /* ATT_H */
