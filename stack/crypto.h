/*
 * BLE PHY Encryption Demo — AES-ECB / AES-CCM 硬件加密
 *
 * 使用 nRF52 ECB 和 CCM 外设直接实现 BLE LE 加密所需的密码学功能:
 *   - ECB: 会话密钥推导 SK = e(LTK, SKD)
 *   - CCM: PDU 加密/解密 (含 4 字节 MIC)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CRYPTO_H
#define CRYPTO_H

#include <stdint.h>
#include <stdbool.h>

/* Forward declaration — 具体定义在 hal/ccm.h */
struct ccm;

/*
 * AES-128 ECB 加密 (nRF52 ECB 硬件)
 *
 * 输入: key_le, plaintext_le — 16 字节, 小端 (ARM 自然字节序)
 * 输出: ciphertext_be — 16 字节, 大端 (CCM 硬件所需格式)
 *
 * 注意: nRF52 CCM 期望密钥以大端存储, 所以输出直接是大端。
 */
void crypto_ecb_encrypt(const uint8_t key_le[16],
			const uint8_t plaintext_le[16],
			uint8_t ciphertext_be[16]);

/*
 * AES-128 ECB 加密 — 小端输入/输出 (SMP 密码函数使用)
 *
 * 与 crypto_ecb_encrypt 相同, 但输出为小端。
 * 等价于 Zephyr 的 bt_encrypt_le()。
 */
void crypto_aes128_le(const uint8_t key_le[16],
		      const uint8_t in_le[16],
		      uint8_t out_le[16]);

/*
 * AES-CCM 加密 (nRF52 CCM 硬件, BLE Extended 模式)
 *
 * config: struct ccm (key/counter/direction/iv)
 * pdu_in: 明文 PDU (Header[2] + Payload[len])
 * pdu_out: 密文 PDU (Header[2] + EncPayload[len] + MIC[4])
 *          输出的 len 字段 = 输入 len + 4
 *
 * 返回: true 如果加密成功
 */
bool crypto_ccm_encrypt(struct ccm *config,
			const void *pdu_in,
			void *pdu_out);

/*
 * AES-CCM 解密 (nRF52 CCM 硬件, BLE Extended 模式)
 *
 * config: struct ccm (key/counter/direction/iv)
 * pdu_in: 密文 PDU (Header[2] + EncPayload[len-4] + MIC[4])
 * pdu_out: 明文 PDU (Header[2] + Payload[len-4])
 *          输出的 len 字段 = 输入 len - 4
 * mic_ok: 输出参数, true 如果 MIC 校验通过
 *
 * 返回: true 如果解密操作完成 (需额外检查 mic_ok)
 */
bool crypto_ccm_decrypt(struct ccm *config,
			const void *pdu_in,
			void *pdu_out,
			bool *mic_ok);

#endif /* CRYPTO_H */
