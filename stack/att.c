/*
 * BLE PHY ATT Demo — Attribute Protocol Server
 *
 * 最小 ATT Server 实现, 处理 GATT Client 的服务发现和属性读写:
 *
 *   Client (手机/nRF Connect)         Server (本 Demo)
 *     │                                   │
 *     │  MTU Exchange                     │
 *     │──────────────────────────────────►│
 *     │◄──────────────────────────────────│
 *     │                                   │
 *     │  Read By Group Type (0x2800)      │  → 服务发现
 *     │──────────────────────────────────►│
 *     │◄──────────────────────────────────│
 *     │                                   │
 *     │  Read By Type (0x2803)            │  → 特征发现
 *     │──────────────────────────────────►│
 *     │◄──────────────────────────────────│
 *     │                                   │
 *     │  Read / Write                     │  → 属性读写
 *     │──────────────────────────────────►│
 *     │◄──────────────────────────────────│
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "att.h"
#include "gatt_db.h"
#include "l2cap.h"

static uint16_t att_mtu = ATT_MTU_DEFAULT;

/*===========================================================================
 * ATT 帧发送 (通过 L2CAP CID=0x0004)
 *===========================================================================*/
static void att_send(const uint8_t *data, uint16_t len)
{
	uint8_t frame[L2CAP_HDR_SIZE + 64];

	if (len + L2CAP_HDR_SIZE > sizeof(frame)) {
		printk("[ATT] TX too large: %u\n", len);
		return;
	}

	sys_put_le16(len, &frame[0]);
	sys_put_le16(L2CAP_CID_ATT, &frame[2]);
	memcpy(&frame[L2CAP_HDR_SIZE], data, len);

	int queued = l2cap_tx_enqueue(frame, L2CAP_HDR_SIZE + len);
	if (queued <= 0) {
		printk("[ATT] TX queue full!\n");
	}
}

/*===========================================================================
 * Error Response
 *===========================================================================*/
static void att_send_error(uint8_t req_opcode, uint16_t handle, uint8_t error)
{
	uint8_t pdu[5];
	pdu[0] = ATT_OP_ERROR_RSP;
	pdu[1] = req_opcode;
	sys_put_le16(handle, &pdu[2]);
	pdu[4] = error;
	att_send(pdu, 5);
	printk("[ATT] TX: Error RSP (req=0x%02X handle=0x%04X err=0x%02X)\n",
	       req_opcode, handle, error);
}

/*===========================================================================
 * Exchange MTU (0x02 → 0x03)
 *===========================================================================*/
static void att_handle_mtu_req(const uint8_t *payload, uint16_t len)
{
	if (len < 3) {
		att_send_error(ATT_OP_MTU_REQ, 0x0000, ATT_ERR_INVALID_PDU);
		return;
	}

	uint16_t client_mtu = sys_get_le16(&payload[1]);
	printk("[ATT] RX: MTU Exchange Request (client_mtu=%u)\n", client_mtu);

	if (client_mtu < ATT_MTU_DEFAULT) {
		client_mtu = ATT_MTU_DEFAULT;
	}

	/* 我们的 Server MTU 固定为默认值 (23), 受 LL PDU 长度限制 */
	uint16_t server_mtu = ATT_MTU_DEFAULT;
	att_mtu = (client_mtu < server_mtu) ? client_mtu : server_mtu;

	uint8_t rsp[3];
	rsp[0] = ATT_OP_MTU_RSP;
	sys_put_le16(server_mtu, &rsp[1]);
	att_send(rsp, 3);
	printk("[ATT] TX: MTU Exchange Response (server_mtu=%u, effective=%u)\n",
	       server_mtu, att_mtu);
}

/*===========================================================================
 * Find Information (0x04 → 0x05)
 *
 * 返回 [handle, uuid] 映射, 用于 Descriptor 发现
 *===========================================================================*/
static void att_handle_find_info_req(const uint8_t *payload, uint16_t len)
{
	if (len < 5) {
		att_send_error(ATT_OP_FIND_INFO_REQ, 0x0000, ATT_ERR_INVALID_PDU);
		return;
	}

	uint16_t start = sys_get_le16(&payload[1]);
	uint16_t end   = sys_get_le16(&payload[3]);

	printk("[ATT] RX: Find Information (0x%04X..0x%04X)\n", start, end);

	if (start == 0 || start > end) {
		att_send_error(ATT_OP_FIND_INFO_REQ, start, ATT_ERR_INVALID_HANDLE);
		return;
	}

	/* 构造响应: [opcode(1)] [format(1)] [handle(2)+uuid16(2)]... */
	uint8_t rsp[ATT_MTU_DEFAULT];
	uint16_t offset = 2;  /* 跳过 opcode + format */
	rsp[0] = ATT_OP_FIND_INFO_RSP;
	rsp[1] = 0x01;  /* format=UUID16 (我们只用 16-bit UUID) */

	for (uint16_t i = 0; i < gatt_db_size; i++) {
		if (gatt_db[i].handle < start || gatt_db[i].handle > end) {
			continue;
		}
		if (offset + 4 > att_mtu) {
			break;
		}
		sys_put_le16(gatt_db[i].handle, &rsp[offset]);
		sys_put_le16(gatt_db[i].type_uuid, &rsp[offset + 2]);
		offset += 4;
	}

	if (offset == 2) {
		att_send_error(ATT_OP_FIND_INFO_REQ, start, ATT_ERR_ATTR_NOT_FOUND);
		return;
	}

	att_send(rsp, offset);
	printk("[ATT] TX: Find Information Response (%u entries)\n",
	       (offset - 2) / 4);
}

/*===========================================================================
 * Read By Type (0x08 → 0x09)
 *
 * 用于 Characteristic Discovery (uuid=0x2803) 和其他类型查询
 *===========================================================================*/
static void att_handle_read_type_req(const uint8_t *payload, uint16_t len)
{
	if (len < 7) {
		att_send_error(ATT_OP_READ_TYPE_REQ, 0x0000, ATT_ERR_INVALID_PDU);
		return;
	}

	uint16_t start = sys_get_le16(&payload[1]);
	uint16_t end   = sys_get_le16(&payload[3]);
	uint16_t uuid  = sys_get_le16(&payload[5]);  /* 只支持 UUID16 */

	printk("[ATT] RX: Read By Type (0x%04X..0x%04X uuid=0x%04X)\n",
	       start, end, uuid);

	if (start == 0 || start > end) {
		att_send_error(ATT_OP_READ_TYPE_REQ, start, ATT_ERR_INVALID_HANDLE);
		return;
	}

	/* 构造响应: [opcode(1)] [len(1)] [handle(2)+value(...)]... */
	uint8_t rsp[ATT_MTU_DEFAULT];
	uint16_t offset = 2;  /* 跳过 opcode + len */
	rsp[0] = ATT_OP_READ_TYPE_RSP;
	uint8_t item_len = 0;  /* 每个条目的长度 (handle + value) */

	for (uint16_t i = 0; i < gatt_db_size; i++) {
		if (gatt_db[i].handle < start || gatt_db[i].handle > end) {
			continue;
		}
		if (gatt_db[i].type_uuid != uuid) {
			continue;
		}

		/* 每个条目: handle(2B) + value */
		uint8_t this_len = 2 + gatt_db[i].value_len;

		if (item_len == 0) {
			item_len = this_len;
		} else if (this_len != item_len) {
			/* 长度不一致的条目不能放在同一个响应中 */
			break;
		}

		if (offset + item_len > att_mtu) {
			break;
		}

		sys_put_le16(gatt_db[i].handle, &rsp[offset]);
		memcpy(&rsp[offset + 2], gatt_db[i].value, gatt_db[i].value_len);
		offset += item_len;
	}

	if (offset == 2) {
		att_send_error(ATT_OP_READ_TYPE_REQ, start, ATT_ERR_ATTR_NOT_FOUND);
		return;
	}

	rsp[1] = item_len;
	att_send(rsp, offset);
	printk("[ATT] TX: Read By Type Response (item_len=%u, %u entries)\n",
	       item_len, (offset - 2) / item_len);
}

/*===========================================================================
 * Read By Group Type (0x10 → 0x11)
 *
 * 用于 Service Discovery (uuid=0x2800 或 0x2801)
 *===========================================================================*/
static void att_handle_read_group_type_req(const uint8_t *payload, uint16_t len)
{
	if (len < 7) {
		att_send_error(ATT_OP_READ_GROUP_REQ, 0x0000, ATT_ERR_INVALID_PDU);
		return;
	}

	uint16_t start = sys_get_le16(&payload[1]);
	uint16_t end   = sys_get_le16(&payload[3]);
	uint16_t uuid  = sys_get_le16(&payload[5]);

	printk("[ATT] RX: Read By Group Type (0x%04X..0x%04X uuid=0x%04X)\n",
	       start, end, uuid);

	if (start == 0 || start > end) {
		att_send_error(ATT_OP_READ_GROUP_REQ, start, ATT_ERR_INVALID_HANDLE);
		return;
	}

	/* 只允许 Primary Service 和 Secondary Service */
	if (uuid != UUID_PRIMARY_SERVICE && uuid != UUID_SECONDARY_SERVICE) {
		att_send_error(ATT_OP_READ_GROUP_REQ, start,
			       ATT_ERR_UNSUPPORTED_GROUP_TYPE);
		return;
	}

	/* 构造响应: [opcode(1)] [len(1)] [start(2)+end(2)+value(...)]... */
	uint8_t rsp[ATT_MTU_DEFAULT];
	uint16_t offset = 2;
	rsp[0] = ATT_OP_READ_GROUP_RSP;
	uint8_t item_len = 0;

	for (uint16_t i = 0; i < gatt_db_size; i++) {
		if (gatt_db[i].handle < start || gatt_db[i].handle > end) {
			continue;
		}
		if (gatt_db[i].type_uuid != uuid) {
			continue;
		}

		/* 每个条目: start_handle(2B) + end_handle(2B) + service_uuid */
		uint16_t group_end = gatt_db_group_end(i);
		uint8_t this_len = 4 + gatt_db[i].value_len;

		if (item_len == 0) {
			item_len = this_len;
		} else if (this_len != item_len) {
			break;
		}

		if (offset + item_len > att_mtu) {
			break;
		}

		sys_put_le16(gatt_db[i].handle, &rsp[offset]);
		sys_put_le16(group_end, &rsp[offset + 2]);
		memcpy(&rsp[offset + 4], gatt_db[i].value, gatt_db[i].value_len);
		offset += item_len;
	}

	if (offset == 2) {
		att_send_error(ATT_OP_READ_GROUP_REQ, start,
			       ATT_ERR_ATTR_NOT_FOUND);
		return;
	}

	rsp[1] = item_len;
	att_send(rsp, offset);
	printk("[ATT] TX: Read By Group Type Response (item_len=%u, %u entries)\n",
	       item_len, (offset - 2) / item_len);
}

/*===========================================================================
 * Read (0x0A → 0x0B)
 *===========================================================================*/
static void att_handle_read_req(const uint8_t *payload, uint16_t len)
{
	if (len < 3) {
		att_send_error(ATT_OP_READ_REQ, 0x0000, ATT_ERR_INVALID_PDU);
		return;
	}

	uint16_t handle = sys_get_le16(&payload[1]);

	printk("[ATT] RX: Read Request (handle=0x%04X)\n", handle);

	struct gatt_attr *attr = gatt_db_find(handle);
	if (!attr) {
		att_send_error(ATT_OP_READ_REQ, handle, ATT_ERR_INVALID_HANDLE);
		return;
	}

	if (!(attr->perm & ATT_PERM_READ)) {
		att_send_error(ATT_OP_READ_REQ, handle, ATT_ERR_READ_NOT_PERMITTED);
		return;
	}

	/* 构造响应: [opcode(1)] [value(最多 ATT_MTU-1)] */
	uint8_t rsp[ATT_MTU_DEFAULT];
	rsp[0] = ATT_OP_READ_RSP;

	uint16_t copy_len = attr->value_len;
	if (copy_len > att_mtu - 1) {
		copy_len = att_mtu - 1;
	}

	memcpy(&rsp[1], attr->value, copy_len);
	att_send(rsp, 1 + copy_len);
	printk("[ATT] TX: Read Response (len=%u)\n", copy_len);
}

/*===========================================================================
 * Write (0x12 → 0x13)
 *===========================================================================*/
static void att_handle_write_req(const uint8_t *payload, uint16_t len)
{
	if (len < 3) {
		att_send_error(ATT_OP_WRITE_REQ, 0x0000, ATT_ERR_INVALID_PDU);
		return;
	}

	uint16_t handle = sys_get_le16(&payload[1]);

	printk("[ATT] RX: Write Request (handle=0x%04X len=%u)\n",
	       handle, len - 3);

	struct gatt_attr *attr = gatt_db_find(handle);
	if (!attr) {
		att_send_error(ATT_OP_WRITE_REQ, handle, ATT_ERR_INVALID_HANDLE);
		return;
	}

	if (!(attr->perm & ATT_PERM_WRITE)) {
		att_send_error(ATT_OP_WRITE_REQ, handle, ATT_ERR_WRITE_NOT_PERMITTED);
		return;
	}

	uint16_t write_len = len - 3;
	const uint8_t *write_data = &payload[3];

	/* 检查值缓冲区是否可写 */
	if (attr->value_max_len == 0) {
		att_send_error(ATT_OP_WRITE_REQ, handle, ATT_ERR_WRITE_NOT_PERMITTED);
		return;
	}

	if (write_len > attr->value_max_len) {
		att_send_error(ATT_OP_WRITE_REQ, handle, 0x0D); /* Invalid Attr Len */
		return;
	}

	/* 实际写入属性值 */
	memcpy(attr->value, write_data, write_len);
	attr->value_len = write_len;

	printk("[ATT] Write data:");
	for (uint16_t i = 0; i < write_len && i < 16; i++) {
		printk(" %02X", write_data[i]);
	}
	printk("\n");

	/* 特殊处理: CCC 写入 */
	if (attr->type_uuid == UUID_CCC_DESCRIPTOR) {
		uint16_t ccc_val = (write_len >= 2)
			? sys_get_le16(write_data) : write_data[0];
		printk("[ATT] CCC updated: Notify=%s Indicate=%s\n",
		       (ccc_val & 0x01) ? "ON" : "OFF",
		       (ccc_val & 0x02) ? "ON" : "OFF");
	}

	/* 发送空 Write Response */
	uint8_t rsp[1] = { ATT_OP_WRITE_RSP };
	att_send(rsp, 1);
	printk("[ATT] TX: Write Response\n");
}

/*===========================================================================
 * Notification (Server → Client)
 *===========================================================================*/
void att_notify(uint16_t handle, const uint8_t *value, uint16_t len)
{
	uint8_t frame[L2CAP_HDR_SIZE + ATT_MTU_DEFAULT];
	uint16_t att_len = 3 + len;

	if (att_len > att_mtu) {
		att_len = att_mtu;
		len = att_mtu - 3;
	}

	/* L2CAP header */
	sys_put_le16(att_len, &frame[0]);
	sys_put_le16(L2CAP_CID_ATT, &frame[2]);

	/* ATT notification */
	frame[4] = ATT_OP_NOTIFY;
	sys_put_le16(handle, &frame[5]);
	memcpy(&frame[7], value, len);

	int queued = l2cap_tx_enqueue(frame, L2CAP_HDR_SIZE + att_len);
	if (queued > 0) {
		printk("[ATT] TX: Notification (handle=0x%04X len=%u)\n",
		       handle, len);
	}
}

/*===========================================================================
 * ATT PDU 派发
 *===========================================================================*/
void att_handle(const uint8_t *payload, uint16_t len)
{
	if (len < 1) {
		return;
	}

	uint8_t opcode = payload[0];

	switch (opcode) {
	case ATT_OP_MTU_REQ:
		att_handle_mtu_req(payload, len);
		break;

	case ATT_OP_FIND_INFO_REQ:
		att_handle_find_info_req(payload, len);
		break;

	case ATT_OP_READ_TYPE_REQ:
		att_handle_read_type_req(payload, len);
		break;

	case ATT_OP_READ_GROUP_REQ:
		att_handle_read_group_type_req(payload, len);
		break;

	case ATT_OP_READ_REQ:
		att_handle_read_req(payload, len);
		break;

	case ATT_OP_WRITE_REQ:
		att_handle_write_req(payload, len);
		break;

	case ATT_OP_WRITE_CMD:
		/* Write Without Response — 不需要回复 */
		printk("[ATT] RX: Write Command (handle=0x%04X)\n",
		       len >= 3 ? sys_get_le16(&payload[1]) : 0);
		break;

	case ATT_OP_CONFIRM:
		/* Handle Value Confirmation (对 Indication 的回复) */
		printk("[ATT] RX: Confirmation\n");
		break;

	default:
		printk("[ATT] RX: Unsupported opcode 0x%02X\n", opcode);
		att_send_error(opcode, 0x0000, ATT_ERR_NOT_SUPPORTED);
		break;
	}
}

void att_init(void)
{
	att_mtu = ATT_MTU_DEFAULT;
	printk("[ATT] Initialized (MTU=%u, DB size=%u attrs)\n",
	       att_mtu, gatt_db_size);
}
