/*
 * BLE PHY SMP Demo — Security Manager Protocol
 *
 * LE Legacy Pairing (Just Works) 实现:
 *   - Pairing Request/Response
 *   - Pairing Confirm/Random (c1 函数)
 *   - STK 生成 (s1 函数)
 *   - 触发 LL 加密
 *   - 密钥分发 (LTK + EDIV + Rand)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SMP_H
#define SMP_H

#include "ble_common.h"

/*
 * 处理 CID=0x0006 的 SMP 帧
 * payload: 指向 L2CAP payload (不含 L2CAP header)
 * len: payload 长度
 */
void smp_handle(const uint8_t *payload, uint16_t len);

/*
 * 加密建立后调用 — 分发密钥 (Encryption Information + Master Identification)
 */
void smp_distribute_keys(void);

/*
 * 初始化 SMP 状态
 */
void smp_init(void);

#endif /* SMP_H */
