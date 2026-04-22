/*
 * BLE Slave-Initiated LL Procedures & GATT Notification 调度
 *
 * 本模块在 08_phy_ll_procedures 引入 (与 anchor.c 一起处理 Instant 相关),
 * 11_phy_slave_procedures 中通过 BLE_FEATURE_SLAVE_CPR / SLAVE_TERMINATE
 * 开关激活 Slave 主动过程, 16_phy_gatt 中通过 BLE_FEATURE_GATT_NOTIFY
 * 激活 GATT counter 通知。
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LL_PROCEDURE_H
#define LL_PROCEDURE_H

#include "ble_common.h"

/*
 * ll_procedure_init — conn_loop() 开头调用一次
 *
 * 根据 conn_interval_us 计算 CPR / TERMINATE_IND 触发阈值
 * (以连接事件数计)。
 */
void ll_procedure_init(uint32_t conn_interval_us);

/*
 * ll_procedure_tick — 每个连接事件开头调用
 *
 * 检查是否到达 CPR / TERMINATE_IND 的触发阈值, 若是则设置对应的
 * proc_slave_cpr / proc_slave_term 请求。
 *
 * 空操作当 BLE_FEATURE_SLAVE_CPR=0 且 BLE_FEATURE_SLAVE_TERMINATE=0。
 */
void ll_procedure_tick(void);

/*
 * ll_procedure_gatt_tick — 每个连接事件末尾调用
 *
 * 每隔约 1 秒递增 GATT counter 特征值, 若 CCC 通知已使能则发送通知。
 * 空操作当 BLE_FEATURE_GATT_NOTIFY=0。
 */
void ll_procedure_gatt_tick(uint32_t conn_interval_us);

/*
 * ll_procedure_summary — 连接结束后在 summary 中追加 Slave Procedures 段
 *
 * 当 BLE_FEATURE_SLAVE_CPR=0 且 BLE_FEATURE_SLAVE_TERMINATE=0 时为空操作。
 */
void ll_procedure_summary(void);

#endif /* LL_PROCEDURE_H */
