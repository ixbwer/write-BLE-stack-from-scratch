/*
 * BLE PHY GATT Demo — GATT 属性数据库
 *
 * 静态属性表:
 *
 *   Handle  Type     Value                              Permission
 *   ------  ----     -----                              ----------
 *   0x0001  0x2800   0x1800 (GAP Service)               R
 *   0x0002  0x2803   {R, 0x0003, 0x2A00}                R
 *   0x0003  0x2A00   "GATT" (Device Name)               R
 *   0x0004  0x2803   {R, 0x0005, 0x2A01}                R
 *   0x0005  0x2A01   0x0000 (Appearance)                R
 *   0x0006  0x2800   0x1801 (GATT Service)              R
 *   0x0007  0x2800   0xFF00 (Custom Service)            R
 *   0x0008  0x2803   {R|N, 0x0009, 0xFF01}              R
 *   0x0009  0xFF01   counter_val (2B)                   R
 *   0x000A  0x2902   ccc_counter (2B)                   R+W
 *   0x000B  0x2803   {R|W, 0x000C, 0xFF02}              R
 *   0x000C  0xFF02   led_val (1B)                       R+W
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gatt_db.h"

/* Service UUID 值 (小端 2 字节) */
static uint8_t val_gap_svc[]    = { 0x00, 0x18 };
static uint8_t val_gatt_svc[]   = { 0x01, 0x18 };
static uint8_t val_custom_svc[] = { 0x00, 0xFF };

/* Characteristic Declaration 值 */
static uint8_t val_chrc_device_name[] = {
	CHRC_PROP_READ, 0x03, 0x00, 0x00, 0x2A,
};
static uint8_t val_chrc_appearance[] = {
	CHRC_PROP_READ, 0x05, 0x00, 0x01, 0x2A,
};
static uint8_t val_chrc_counter[] = {
	CHRC_PROP_READ | CHRC_PROP_NOTIFY, 0x09, 0x00, 0x01, 0xFF,
};
static uint8_t val_chrc_led[] = {
	CHRC_PROP_READ | CHRC_PROP_WRITE, 0x0C, 0x00, 0x02, 0xFF,
};

/* Characteristic Value 数据 */
static uint8_t val_device_name[] = { 'G', 'A', 'T', 'T' };
static uint8_t val_appearance[]  = { 0x00, 0x00 };

/* 可写值 (extern 暴露给 main.c / att.c) */
uint8_t gatt_counter_val[2] = { 0x00, 0x00 };
uint8_t gatt_ccc_counter[2] = { 0x00, 0x00 };
uint8_t gatt_led_val[1]     = { 0x00 };

/*
 * 完整属性数据库 (12 个属性)
 */
struct gatt_attr gatt_db[] = {
	/* === GAP Service (0x1800) === */
	{ .handle = 0x0001, .type_uuid = UUID_PRIMARY_SERVICE,
	  .perm = ATT_PERM_READ,
	  .value = val_gap_svc, .value_len = 2, .value_max_len = 0 },

	{ .handle = 0x0002, .type_uuid = UUID_CHARACTERISTIC,
	  .perm = ATT_PERM_READ,
	  .value = val_chrc_device_name, .value_len = 5, .value_max_len = 0 },

	{ .handle = 0x0003, .type_uuid = UUID_DEVICE_NAME,
	  .perm = ATT_PERM_READ,
	  .value = val_device_name, .value_len = 4, .value_max_len = 0 },

	{ .handle = 0x0004, .type_uuid = UUID_CHARACTERISTIC,
	  .perm = ATT_PERM_READ,
	  .value = val_chrc_appearance, .value_len = 5, .value_max_len = 0 },

	{ .handle = 0x0005, .type_uuid = UUID_APPEARANCE,
	  .perm = ATT_PERM_READ,
	  .value = val_appearance, .value_len = 2, .value_max_len = 0 },

	/* === GATT Service (0x1801) === */
	{ .handle = 0x0006, .type_uuid = UUID_PRIMARY_SERVICE,
	  .perm = ATT_PERM_READ,
	  .value = val_gatt_svc, .value_len = 2, .value_max_len = 0 },

	/* === Custom Service (0xFF00) === */
	{ .handle = 0x0007, .type_uuid = UUID_PRIMARY_SERVICE,
	  .perm = ATT_PERM_READ,
	  .value = val_custom_svc, .value_len = 2, .value_max_len = 0 },

	{ .handle = 0x0008, .type_uuid = UUID_CHARACTERISTIC,
	  .perm = ATT_PERM_READ,
	  .value = val_chrc_counter, .value_len = 5, .value_max_len = 0 },

	{ .handle = 0x0009, .type_uuid = UUID_CUSTOM_COUNTER,
	  .perm = ATT_PERM_READ,
	  .value = gatt_counter_val, .value_len = 2, .value_max_len = 2 },

	{ .handle = 0x000A, .type_uuid = UUID_CCC_DESCRIPTOR,
	  .perm = ATT_PERM_READ | ATT_PERM_WRITE,
	  .value = gatt_ccc_counter, .value_len = 2, .value_max_len = 2 },

	{ .handle = 0x000B, .type_uuid = UUID_CHARACTERISTIC,
	  .perm = ATT_PERM_READ,
	  .value = val_chrc_led, .value_len = 5, .value_max_len = 0 },

	{ .handle = 0x000C, .type_uuid = UUID_CUSTOM_LED,
	  .perm = ATT_PERM_READ | ATT_PERM_WRITE,
	  .value = gatt_led_val, .value_len = 1, .value_max_len = 1 },
};

const uint16_t gatt_db_size = sizeof(gatt_db) / sizeof(gatt_db[0]);

struct gatt_attr *gatt_db_find(uint16_t handle)
{
	for (uint16_t i = 0; i < gatt_db_size; i++) {
		if (gatt_db[i].handle == handle) {
			return &gatt_db[i];
		}
	}
	return NULL;
}

uint16_t gatt_db_group_end(uint16_t svc_index)
{
	/* 从 svc_index+1 开始扫描, 找到下一个 Service 声明 */
	for (uint16_t i = svc_index + 1; i < gatt_db_size; i++) {
		if (gatt_db[i].type_uuid == UUID_PRIMARY_SERVICE ||
		    gatt_db[i].type_uuid == UUID_SECONDARY_SERVICE) {
			return gatt_db[i - 1].handle;
		}
	}
	/* 没有下一个 Service → 最后一个属性就是 end */
	return gatt_db[gatt_db_size - 1].handle;
}
