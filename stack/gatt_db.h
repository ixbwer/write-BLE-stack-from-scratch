/*
 * BLE PHY GATT Demo — GATT 属性数据库
 *
 * 手工构造的静态属性表:
 *   - GAP Service (0x1800): Device Name + Appearance
 *   - GATT Service (0x1801): 仅声明
 *   - Custom Service (0xFF00): Counter (R+Notify) + LED (R+W)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GATT_DB_H
#define GATT_DB_H

#include "att.h"

/* Custom Service UUIDs */
#define UUID_CUSTOM_SERVICE   0xFF00
#define UUID_CUSTOM_COUNTER   0xFF01
#define UUID_CUSTOM_LED       0xFF02

/* Writable attribute maximum value length */
#define GATT_WRITABLE_MAX_LEN 20

/*
 * 属性结构 (支持可写属性):
 *   handle — 属性句柄 (从 0x0001 开始)
 *   type_uuid — 属性类型 (16-bit UUID)
 *   perm — 权限位图
 *   value — 指向属性值的指针 (const 或可写缓冲区)
 *   value_len — 当前属性值长度 (可变)
 *   value_max_len — 值缓冲区最大长度 (0=只读 const)
 */
struct gatt_attr {
	uint16_t      handle;
	uint16_t      type_uuid;
	uint8_t       perm;
	uint8_t       *value;
	uint16_t      value_len;
	uint16_t      value_max_len;
};

/* 全局属性数据库 */
extern struct gatt_attr gatt_db[];
extern const uint16_t gatt_db_size;

/* 根据 handle 查找属性 (返回可写指针, 支持 Write Request 更新 value_len) */
struct gatt_attr *gatt_db_find(uint16_t handle);

/* 获取 Service group 的结束 handle */
uint16_t gatt_db_group_end(uint16_t svc_index);

/* Custom service 的可写值 (extern 供 main.c 访问) */
extern uint8_t gatt_counter_val[2];
extern uint8_t gatt_ccc_counter[2];
extern uint8_t gatt_led_val[1];

/* Handle 常量 (供 notification 使用) */
#define HANDLE_COUNTER_VAL   0x0009
#define HANDLE_COUNTER_CCC   0x000A
#define HANDLE_LED_VAL       0x000C

#endif /* GATT_DB_H */
