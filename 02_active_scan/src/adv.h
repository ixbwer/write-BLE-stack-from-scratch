/*
 * BLE PHY 软件 TIFS Demo — 广播状态机
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ADV_H
#define ADV_H

#include "ble_common.h"

/* 广播工作处理函数 (系统工作队列回调) */
void adv_worker(struct k_work *work);

/* 定时器到期回调 */
void adv_timer_expiry(struct k_timer *timer);

/* 初始化调度器组件 (k_work + k_timer) */
void adv_init(void);

/* 启动广播定时器 */
void adv_start(void);

#endif /* ADV_H */
