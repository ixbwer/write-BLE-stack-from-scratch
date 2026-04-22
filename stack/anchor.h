/*
 * BLE 连接锚点: 窗口扩展 (WW)、漂移修正、Supervision Timeout 辅助
 *
 * 本模块在 04_phy_connection 引入, 之后所有连接 demo 共用。
 *
 * 每个连接事件的时序由三个量维护:
 *   next_event_rtc         — 下一事件开始的 RTC0 tick
 *   next_event_remainder_ps — tick 内的亚 tick 余量 (picoseconds)
 *   ww_event_us            — 本事件的窗口扩展量 (连续 miss 越多越大)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ANCHOR_H
#define ANCHOR_H

#include "ble_common.h"

/*
 * 连接会话锚点状态 — conn_loop() 的本地时序上下文
 *
 * 由 anchor_session_init() 初始化, 之后由 conn_loop() 与 anchor_* 辅助函数
 * 一起维护。所有时序计算 (窗口扩展、漂移修正、Supervision Timeout) 都只
 * 读写此结构体, 不再把这些细节暴露给 conn_loop。
 */
struct anchor_session {
	/* 时序参数 (可能被 LL_CONNECTION_UPDATE_IND 改写) */
	uint32_t conn_interval_us;
	uint32_t interval_ticks;
	uint32_t interval_remainder_ps;
	uint32_t supervision_timeout_us;

	/* 常量 (仅在 session_init 时设置一次) */
	uint32_t combined_sca_ppm;
	uint32_t ps_per_tick;

	/* 滚动状态 */
	uint32_t next_event_rtc;
	uint32_t next_event_remainder_ps;
	uint32_t ww_event_us;
	uint32_t win_size_event_us;
	uint32_t last_rx_rtc;
};

/*
 * anchor_session_init — 连接进入时初始化一次
 *
 * 1. memset 全局 `anchor` 并根据 conn_params / connect_end_rtc 计算:
 *      ww_periodic_us, ww_max_us, win_size_event_us, combined_sca_ppm,
 *      interval_ticks, interval_remainder_ps, supervision_timeout_us,
 *      初始 next_event_rtc / next_event_remainder_ps。
 * 2. 调用 anchor_catchup() 跳过启动延迟导致的过时锚点。
 *
 * 返回后 *s 即可直接用于主循环。
 */
void anchor_session_init(struct anchor_session *s);

/*
 * anchor_prepare_event — 本事件开始前的时序准备 (监听路径)
 *
 * 累加 ww_event_us、写入 anchor.ww_current_us 与 anchor.current_anchor_rtc,
 * 返回应传给 conn_event_run() 的 hcto_add 参数。
 */
uint32_t anchor_prepare_event(struct anchor_session *s);

/*
 * anchor_skip_event — slave latency 跳过本事件
 *
 * 累加 ww_event_us, 推进 chan_sel_1() / conn_event_counter 以保持与 Master 同步。
 */
void anchor_skip_event(struct anchor_session *s);

/*
 * anchor_on_rx_ok — 成功 RX 后的锚点维护
 *
 * 漂移修正 next_event_rtc、更新 last_rx_rtc / anchor.last_rx_anchor_rtc /
 * anchor.anchor_update_count, 清零 consecutive_misses, 重置 ww_event / win_size。
 */
void anchor_on_rx_ok(struct anchor_session *s);

/*
 * anchor_on_rx_miss — 一次 RX 丢失后的锚点维护
 *
 * 累加 consecutive_misses 并更新 max; 调用 anchor_supervision_check()。
 * 返回 true 表示已超过 Supervision Timeout, 调用方应跳出主循环。
 */
bool anchor_on_rx_miss(struct anchor_session *s);

/*
 * anchor_apply_conn_update — 检查 LL_CONNECTION_UPDATE_IND 的 Instant
 *
 * 若 conn_event_counter == instant, 把新参数写入 conn_params,
 * 并更新 *conn_interval_us / *interval_ticks / *interval_remainder_ps /
 * *supervision_timeout_us 及 anchor.ww_periodic_us / ww_max_us。
 *
 * 返回 true 表示参数已切换, 调用方需要重新打印日志。
 */
bool anchor_apply_conn_update(uint32_t *conn_interval_us,
			      uint32_t *interval_ticks,
			      uint32_t *interval_remainder_ps,
			      uint32_t *supervision_timeout_us,
			      uint32_t combined_sca_ppm);

/*
 * anchor_apply_chan_map — 检查 LL_CHANNEL_MAP_IND 的 Instant
 *
 * 若 instant 到达, 把新 chan_map 写入 conn_params。
 * 返回 true 表示 map 已切换。
 */
bool anchor_apply_chan_map(void);

/*
 * anchor_drift_correct — 用实测 RX 时间戳修正下一锚点
 *
 * 在每次成功 RX 后调用。读取 last_rx_aa_us / last_rx_ready_us (全局),
 * 计算漂移并折算到 (*next_event_rtc, *next_event_remainder_ps)。
 */
void anchor_drift_correct(uint32_t *next_event_rtc,
			  uint32_t *next_event_remainder_ps,
			  uint32_t ps_per_tick);

/*
 * anchor_supervision_check — 检查是否超过 Supervision Timeout
 *
 * 若 (next_event_rtc - last_rx_rtc) >= supervision_timeout_us,
 * 设置 conn_terminate_reason=0x08 和 conn_terminated=true。
 * 返回 true 表示已超时。
 */
bool anchor_supervision_check(uint32_t next_event_rtc,
			      uint32_t last_rx_rtc,
			      uint32_t supervision_timeout_us);

/*
 * anchor_catchup — 跳过所有启动延迟导致的过时锚点
 *
 * 在 conn_loop() 第一次进入主循环之前调用一次。
 * 跳过已过时的事件, 同时保持 chan_sel_1() / conn_event_counter 与
 * Master 同步, 并累积 *ww_event_us。
 */
void anchor_catchup(uint32_t *next_event_rtc,
		    uint32_t *next_event_remainder_ps,
		    uint32_t interval_ticks,
		    uint32_t interval_remainder_ps,
		    uint32_t ps_per_tick,
		    uint32_t *ww_event_us);

/*
 * anchor_advance — 推进锚点到下一个连接间隔
 *
 * 在每个连接事件末尾调用, 将 next_event_rtc / next_event_remainder_ps
 * 前移一个 interval。
 */
void anchor_advance(uint32_t *next_event_rtc,
		    uint32_t *next_event_remainder_ps,
		    uint32_t interval_ticks,
		    uint32_t interval_remainder_ps,
		    uint32_t ps_per_tick);

#endif /* ANCHOR_H */
