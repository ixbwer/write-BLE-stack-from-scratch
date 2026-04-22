/*
 * BLE PHY SMP Demo — Security Manager Protocol
 *
 * LE Legacy Pairing (Just Works):
 *   TK = 0 (无 MITM 保护)
 *   c1() = 确认值生成 (AES-128 based)
 *   s1() = STK 推导 (AES-128 based)
 *
 * 配对流程 (Slave 视角):
 *   1. 收到 Pairing Request → 回复 Pairing Response
 *   2. 收到 Mconfirm → 计算 Sconfirm, 回复 Pairing Confirm
 *   3. 收到 Mrand → 验证 Mconfirm, 回复 Pairing Random, 计算 STK
 *   4. Master 发起 LL 加密 (使用 STK 作为临时 LTK)
 *   5. 加密建立后 → 分发密钥 (LTK + EDIV + Rand)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "smp.h"
#include "crypto.h"
#include "l2cap.h"

/* SMP Command Codes */
#define SMP_PAIRING_REQ      0x01
#define SMP_PAIRING_RSP      0x02
#define SMP_PAIRING_CONFIRM  0x03
#define SMP_PAIRING_RANDOM   0x04
#define SMP_PAIRING_FAILED   0x05
#define SMP_ENC_INFO         0x06
#define SMP_MASTER_IDENT     0x07

/* Pairing Failed reason codes */
#define SMP_ERR_CONFIRM_FAILED  0x04
#define SMP_ERR_UNSPECIFIED     0x08

/*===========================================================================
 * SMP 密码函数: c1 (确认值生成)
 *
 * c1(k, r, preq, pres, iat, rat, ia, ra) = e(k, e(k, r XOR p1) XOR p2)
 *
 * p1[16] = iat || rat || preq[7] || pres[7]
 * p2[16] = ra[6] || ia[6] || padding[4]
 *
 * 所有输入为小端字节序。
 *===========================================================================*/
static void smp_c1(const uint8_t k[16], const uint8_t r[16],
		   const uint8_t preq[7], const uint8_t pres[7],
		   uint8_t iat, const uint8_t ia[6],
		   uint8_t rat, const uint8_t ra[6],
		   uint8_t out[16])
{
	uint8_t p1[16], p2[16];

	/* p1 = iat || rat || preq || pres */
	p1[0] = iat;
	p1[1] = rat;
	memcpy(&p1[2], preq, 7);
	memcpy(&p1[9], pres, 7);

	/* p2 = ra || ia || 0x00000000 */
	memcpy(&p2[0], ra, 6);
	memcpy(&p2[6], ia, 6);
	memset(&p2[12], 0, 4);

	/* c1 = e(k, e(k, r XOR p1) XOR p2) */
	for (int i = 0; i < 16; i++) {
		out[i] = r[i] ^ p1[i];
	}
	crypto_aes128_le(k, out, out);
	for (int i = 0; i < 16; i++) {
		out[i] = out[i] ^ p2[i];
	}
	crypto_aes128_le(k, out, out);
}

/*===========================================================================
 * SMP 密码函数: s1 (STK 推导)
 *
 * s1(k, r1, r2) = e(k, r2'[0:7] || r1'[0:7])
 *
 * r1' = r1 低 8 字节 (Srand), r2' = r2 低 8 字节 (Mrand)
 *===========================================================================*/
static void smp_s1(const uint8_t k[16],
		   const uint8_t r1[16], /* Srand */
		   const uint8_t r2[16], /* Mrand */
		   uint8_t out[16])
{
	uint8_t r_prime[16];

	memcpy(&r_prime[0], r2, 8);   /* r2' (Mrand lower 8) */
	memcpy(&r_prime[8], r1, 8);   /* r1' (Srand lower 8) */

	crypto_aes128_le(k, r_prime, out);
}

/*===========================================================================
 * 发送 SMP PDU (通过 L2CAP CID=0x0006)
 *===========================================================================*/
static void smp_send(const uint8_t *data, uint16_t len)
{
	uint8_t frame[L2CAP_HDR_SIZE + 64];

	if (len + L2CAP_HDR_SIZE > sizeof(frame)) {
		printk("[SMP] TX too large: %u\n", len);
		return;
	}

	sys_put_le16(len, &frame[0]);
	sys_put_le16(L2CAP_CID_SMP, &frame[2]);
	memcpy(&frame[L2CAP_HDR_SIZE], data, len);

	int queued = l2cap_tx_enqueue(frame, L2CAP_HDR_SIZE + len);
	if (queued <= 0) {
		printk("[SMP] TX queue full!\n");
	}
}

static void smp_send_pairing_failed(uint8_t reason)
{
	uint8_t pdu[2] = { SMP_PAIRING_FAILED, reason };
	smp_send(pdu, 2);
	smp_state.phase = SMP_PHASE_IDLE;
	printk("[SMP] TX: Pairing Failed (reason=0x%02X)\n", reason);
}

/*===========================================================================
 * Pairing Request 处理
 *===========================================================================*/
static void smp_handle_pairing_req(const uint8_t *payload, uint16_t len)
{
	if (len < 7) {
		smp_send_pairing_failed(SMP_ERR_UNSPECIFIED);
		return;
	}

	printk("[SMP] RX: Pairing Request\n");
	printk("       IO=%u OOB=%u Auth=0x%02X MaxKey=%u "
	       "InitDist=0x%02X RspDist=0x%02X\n",
	       payload[1], payload[2], payload[3],
	       payload[4], payload[5], payload[6]);

	/* 保存完整的 Pairing Request (前 7 字节, 含 code) */
	memcpy(smp_state.preq, payload, 7);

	/* 构造 Pairing Response */
	uint8_t pres[7];
	pres[0] = SMP_PAIRING_RSP;
	pres[1] = 0x03;  /* IO Capability: NoInputNoOutput */
	pres[2] = 0x00;  /* OOB: not present */
	pres[3] = 0x01;  /* AuthReq: Bonding, no MITM, no SC */
	pres[4] = 16;    /* Max Encryption Key Size */
	pres[5] = 0x00;  /* Initiator Key Distribution: none */
	pres[6] = 0x01;  /* Responder Key Distribution: EncKey (LTK) */

	/* 保存 Pairing Response */
	memcpy(smp_state.pres, pres, 7);

	/* 生成 Srand (我们的随机数, 16 字节) */
	uint32_t seed = NRF_RTC0->COUNTER ^ conn_event_counter ^ 0xDEADBEEF;
	for (int i = 0; i < 16; i++) {
		seed = seed * 1103515245 + 12345;
		smp_state.srand[i] = (seed >> 16) & 0xFF;
	}

	/* TK = 0 (Just Works) */
	memset(smp_state.tk, 0, 16);

	smp_send(pres, 7);
	printk("[SMP] TX: Pairing Response\n");

	smp_state.phase = SMP_PHASE_WAIT_CONFIRM;
}

/*===========================================================================
 * Pairing Confirm 处理
 *===========================================================================*/
static void smp_handle_pairing_confirm(const uint8_t *payload, uint16_t len)
{
	if (len < 17 || smp_state.phase != SMP_PHASE_WAIT_CONFIRM) {
		smp_send_pairing_failed(SMP_ERR_UNSPECIFIED);
		return;
	}

	/* 保存 Master 的 confirm value */
	memcpy(smp_state.mconfirm, &payload[1], 16);

	printk("[SMP] RX: Pairing Confirm (Mconfirm)\n");

	/* 计算 Sconfirm = c1(TK, Srand, preq, pres, iat, ia, rat, ra) */
	uint8_t sconfirm[16];

	/* iat=1 (Master, random addr), rat=1 (Slave, random addr)
	 * ia = Master addr (conn_params.peer_addr)
	 * ra = Slave addr (adv_addr) */
	smp_c1(smp_state.tk, smp_state.srand,
	       smp_state.preq, smp_state.pres,
	       1, conn_params.peer_addr,  /* iat, ia (Master) */
	       1, adv_addr,               /* rat, ra (Slave) */
	       sconfirm);

	/* 发送 Sconfirm */
	uint8_t pdu[17];
	pdu[0] = SMP_PAIRING_CONFIRM;
	memcpy(&pdu[1], sconfirm, 16);
	smp_send(pdu, 17);
	printk("[SMP] TX: Pairing Confirm (Sconfirm)\n");

	smp_state.phase = SMP_PHASE_WAIT_RANDOM;
}

/*===========================================================================
 * Pairing Random 处理
 *===========================================================================*/
static void smp_handle_pairing_random(const uint8_t *payload, uint16_t len)
{
	if (len < 17 || smp_state.phase != SMP_PHASE_WAIT_RANDOM) {
		smp_send_pairing_failed(SMP_ERR_UNSPECIFIED);
		return;
	}

	uint8_t mrand[16];
	memcpy(mrand, &payload[1], 16);

	printk("[SMP] RX: Pairing Random (Mrand)\n");

	/* 验证 Mconfirm: 用 Mrand 重新计算 Master 的 confirm value */
	uint8_t expected[16];
	smp_c1(smp_state.tk, mrand,
	       smp_state.preq, smp_state.pres,
	       1, conn_params.peer_addr,
	       1, adv_addr,
	       expected);

	if (memcmp(expected, smp_state.mconfirm, 16) != 0) {
		printk("[SMP] ★ Confirm value mismatch! Pairing failed.\n");
		smp_send_pairing_failed(SMP_ERR_CONFIRM_FAILED);
		return;
	}
	printk("[SMP] Mconfirm verified OK\n");

	/* 发送 Srand */
	uint8_t pdu[17];
	pdu[0] = SMP_PAIRING_RANDOM;
	memcpy(&pdu[1], smp_state.srand, 16);
	smp_send(pdu, 17);
	printk("[SMP] TX: Pairing Random (Srand)\n");

	/* 计算 STK = s1(TK, Srand, Mrand) */
	uint8_t stk[16];
	smp_s1(smp_state.tk, smp_state.srand, mrand, stk);

	printk("[SMP] STK computed: ");
	for (int i = 0; i < 16; i++) {
		printk("%02X", stk[i]);
	}
	printk("\n");

	/* 设置 STK 为 LL 的 LTK, 等待 Master 发起 LL_ENC_REQ */
	memcpy(enc.ltk, stk, 16);
	smp_state.phase = SMP_PHASE_WAIT_ENC;
	printk("[SMP] STK → enc.ltk, waiting for LL_ENC_REQ\n");
}

/*===========================================================================
 * 密钥分发 (加密建立后调用)
 *
 * 分发 Encryption Information (LTK) + Master Identification (EDIV + Rand)
 *===========================================================================*/
void smp_distribute_keys(void)
{
	if (smp_state.phase != SMP_PHASE_WAIT_ENC &&
	    smp_state.phase != SMP_PHASE_KEY_DIST) {
		return;
	}

	smp_state.phase = SMP_PHASE_KEY_DIST;

	/* 生成将要分发的 LTK (用于未来重连) */
	uint32_t seed = NRF_RTC0->COUNTER ^ 0x12345678;
	for (int i = 0; i < 16; i++) {
		seed = seed * 1103515245 + 12345;
		smp_state.dist_ltk[i] = (seed >> 16) & 0xFF;
	}
	/* EDIV 和 Rand: 用于未来标识此 LTK */
	smp_state.dist_ediv = 0;
	memset(smp_state.dist_rand, 0, 8);

	/* Encryption Information (Code=0x06, LTK[16]) */
	uint8_t enc_info[17];
	enc_info[0] = SMP_ENC_INFO;
	memcpy(&enc_info[1], smp_state.dist_ltk, 16);
	smp_send(enc_info, 17);
	printk("[SMP] TX: Encryption Information (LTK distributed)\n");

	/* Master Identification (Code=0x07, EDIV[2] + Rand[8]) */
	uint8_t master_id[11];
	master_id[0] = SMP_MASTER_IDENT;
	sys_put_le16(smp_state.dist_ediv, &master_id[1]);
	memcpy(&master_id[3], smp_state.dist_rand, 8);
	smp_send(master_id, 11);
	printk("[SMP] TX: Master Identification (EDIV=%u)\n",
	       smp_state.dist_ediv);

	smp_state.phase = SMP_PHASE_COMPLETE;
	printk("[SMP] ★ Pairing complete!\n");
}

/*===========================================================================
 * SMP 帧派发 (由 l2cap.c CID=0x0006 调用)
 *===========================================================================*/
void smp_handle(const uint8_t *payload, uint16_t len)
{
	if (len < 1) {
		return;
	}

	uint8_t code = payload[0];

	switch (code) {
	case SMP_PAIRING_REQ:
		smp_handle_pairing_req(payload, len);
		break;

	case SMP_PAIRING_CONFIRM:
		smp_handle_pairing_confirm(payload, len);
		break;

	case SMP_PAIRING_RANDOM:
		smp_handle_pairing_random(payload, len);
		break;

	case SMP_PAIRING_FAILED:
		printk("[SMP] RX: Pairing Failed (reason=0x%02X)\n",
		       len > 1 ? payload[1] : 0xFF);
		smp_state.phase = SMP_PHASE_IDLE;
		break;

	default:
		printk("[SMP] RX: Unknown code 0x%02X\n", code);
		break;
	}
}

void smp_init(void)
{
	memset(&smp_state, 0, sizeof(smp_state));
}
