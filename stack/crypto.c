/*
 * BLE PHY Encryption Demo — AES-ECB / AES-CCM 硬件加密
 *
 * nRF52832 提供两个硬件加速外设:
 *   - NRF_ECB: AES-128 块加密 (16 字节 → 16 字节)
 *   - NRF_CCM: AES-CCM 模式加密 (BLE PDU 级, 含 MIC)
 *
 * ECB 用于会话密钥推导: SK = e(LTK, SKD)
 * CCM 用于连接态 PDU 加密/解密: Encrypt(SK, nonce, PDU) → EncPDU + MIC
 *
 * 关键字节序:
 *   - nRF52 ECB 硬件: key 和 plaintext 使用大端 (MSB 在低地址)
 *   - nRF52 CCM 硬件: key 使用大端, counter/iv 使用小端
 *   - BLE 协议: 所有多字节字段默认小端传输
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "crypto.h"
#include "ble_common.h"

/*===========================================================================
 * AES-128 ECB 加密
 *
 * nRF52 ECB 数据结构 (ECBDATAPTR 指向, 共 48 字节):
 *   Byte 0-15:  Key (大端, MSB 在 byte 0)
 *   Byte 16-31: Plaintext (大端)
 *   Byte 32-47: Ciphertext (大端, 硬件写入)
 *===========================================================================*/
void crypto_ecb_encrypt(const uint8_t key_le[16],
			const uint8_t plaintext_le[16],
			uint8_t ciphertext_be[16])
{
	static uint8_t ecb_data[48] __aligned(4);

	/* 小端 → 大端: 字节翻转 */
	for (int i = 0; i < 16; i++) {
		ecb_data[i]      = key_le[15 - i];
		ecb_data[16 + i] = plaintext_le[15 - i];
	}

	NRF_ECB->ECBDATAPTR = (uint32_t)ecb_data;
	NRF_ECB->EVENTS_ENDECB = 0;
	NRF_ECB->EVENTS_ERRORECB = 0;
	NRF_ECB->TASKS_STARTECB = 1;

	while (NRF_ECB->EVENTS_ENDECB == 0 &&
	       NRF_ECB->EVENTS_ERRORECB == 0) {
		/* busy wait — ECB 完成约 ~14µs */
	}

	/* 输出大端密文 (CCM 硬件直接使用此格式) */
	memcpy(ciphertext_be, &ecb_data[32], 16);
}

/*===========================================================================
 * AES-128 ECB 加密 — 小端输入/输出
 *
 * SMP 的 c1, s1 函数需要小端输出。
 * 内部先调用 ECB 硬件 (大端), 再翻转输出为小端。
 *===========================================================================*/
void crypto_aes128_le(const uint8_t key_le[16],
		      const uint8_t in_le[16],
		      uint8_t out_le[16])
{
	uint8_t out_be[16];

	crypto_ecb_encrypt(key_le, in_le, out_be);

	/* 大端 → 小端: 字节翻转 */
	for (int i = 0; i < 16; i++) {
		out_le[i] = out_be[15 - i];
	}
}

/*===========================================================================
 * AES-CCM 加密 (BLE Extended 模式)
 *
 * nRF52 CCM 配置:
 *   MODE:
 *     MODE = Encryption
 *     LENGTH = Extended (8-bit length field, BLE 4.0+)
 *     DATARATE = 2Mbit (内部处理速度, 与 PHY 无关)
 *
 *   CNFPTR → struct ccm:
 *     key[16]:    会话密钥 (大端)
 *     counter:    uint64_t, 低 39 位 = 包计数器, bit 39 = 方向位
 *     direction:  方向位 (Extended 模式下不使用此字段, 方向在 counter 中)
 *     iv[8]:      初始化向量
 *
 *   INPTR → 明文 PDU:  Header[2] + Payload[len]
 *   OUTPTR → 密文 PDU: Header[2] + EncPayload[len] + MIC[4]
 *   SCRATCHPTR → 43 字节临时缓冲区
 *
 *   SHORTS: ENDKSGEN_CRYPT — 密钥调度完成后自动开始加密
 *===========================================================================*/
bool crypto_ccm_encrypt(struct ccm *config,
			const void *pdu_in,
			void *pdu_out)
{
	static uint8_t scratch[43] __aligned(4);

	NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Disabled;
	NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Enabled;

	uint32_t mode = (CCM_MODE_MODE_Encryption << CCM_MODE_MODE_Pos) &
			CCM_MODE_MODE_Msk;
	mode |= (CCM_MODE_LENGTH_Extended << CCM_MODE_LENGTH_Pos) &
		CCM_MODE_LENGTH_Msk;
	mode |= (CCM_MODE_DATARATE_2Mbit << CCM_MODE_DATARATE_Pos) &
		CCM_MODE_DATARATE_Msk;
	NRF_CCM->MODE = mode;

	NRF_CCM->CNFPTR    = (uint32_t)config;
	NRF_CCM->INPTR     = (uint32_t)pdu_in;
	NRF_CCM->OUTPTR    = (uint32_t)pdu_out;
	NRF_CCM->SCRATCHPTR = (uint32_t)scratch;
	NRF_CCM->SHORTS    = CCM_SHORTS_ENDKSGEN_CRYPT_Msk;

	NRF_CCM->EVENTS_ENDKSGEN = 0;
	NRF_CCM->EVENTS_ENDCRYPT = 0;
	NRF_CCM->EVENTS_ERROR    = 0;

	NRF_CCM->TASKS_KSGEN = 1;

	while (NRF_CCM->EVENTS_ENDCRYPT == 0) {
		if (NRF_CCM->EVENTS_ERROR) {
			NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Disabled;
			return false;
		}
	}

	NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Disabled;
	return true;
}

/*===========================================================================
 * AES-CCM 解密 (BLE Extended 模式)
 *
 * 与加密类似, 但 MODE = Decryption。
 * 解密完成后检查 MICSTATUS 寄存器:
 *   MICSTATUS != 0 → MIC 校验通过
 *   MICSTATUS == 0 → MIC 校验失败 (数据被篡改或密钥错误)
 *===========================================================================*/
bool crypto_ccm_decrypt(struct ccm *config,
			const void *pdu_in,
			void *pdu_out,
			bool *mic_ok)
{
	static uint8_t scratch[43] __aligned(4);

	NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Disabled;
	NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Enabled;

	uint32_t mode = (CCM_MODE_MODE_Decryption << CCM_MODE_MODE_Pos) &
			CCM_MODE_MODE_Msk;
	mode |= (CCM_MODE_LENGTH_Extended << CCM_MODE_LENGTH_Pos) &
		CCM_MODE_LENGTH_Msk;
	mode |= (CCM_MODE_DATARATE_2Mbit << CCM_MODE_DATARATE_Pos) &
		CCM_MODE_DATARATE_Msk;
	NRF_CCM->MODE = mode;

	NRF_CCM->CNFPTR    = (uint32_t)config;
	NRF_CCM->INPTR     = (uint32_t)pdu_in;
	NRF_CCM->OUTPTR    = (uint32_t)pdu_out;
	NRF_CCM->SCRATCHPTR = (uint32_t)scratch;
	NRF_CCM->SHORTS    = CCM_SHORTS_ENDKSGEN_CRYPT_Msk;

	NRF_CCM->EVENTS_ENDKSGEN = 0;
	NRF_CCM->EVENTS_ENDCRYPT = 0;
	NRF_CCM->EVENTS_ERROR    = 0;

	NRF_CCM->TASKS_KSGEN = 1;

	while (NRF_CCM->EVENTS_ENDCRYPT == 0) {
		if (NRF_CCM->EVENTS_ERROR) {
			NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Disabled;
			*mic_ok = false;
			return false;
		}
	}

	*mic_ok = (NRF_CCM->MICSTATUS != 0);
	NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Disabled;
	return true;
}
