#!/usr/bin/env python3
"""
Bumble L2CAP Echo Tester — 测试 07_phy_data_channel Demo 的 L2CAP 回环功能

功能:
  1. 连接到 07_phy_data_channel Demo (或任何支持 L2CAP echo 的 Demo)
  2. 发送若干 L2CAP 帧 (可配置大小和数量)
  3. 接收并验证 echo 返回的数据
  4. 支持单 PDU (≤23 字节) 和多分片 (>23 字节) 两种模式
  5. 输出统计报告: 发送数/接收数/丢失数/延迟

用法:
  # 基础测试: 发送 5 帧, 每帧 16 字节 payload
  python bumble_l2cap_echo.py --transport usb:2FE3:000B --target 66:55:44:33:22:11

  # 多分片测试: 发送 40 字节 payload (需要 2 个 LL PDU)
  python bumble_l2cap_echo.py --transport usb:2FE3:000B --target 66:55:44:33:22:11 --size 40

  # 大量帧测试: 发送 20 帧, 每帧间隔 500ms
  python bumble_l2cap_echo.py --transport usb:2FE3:000B --target 66:55:44:33:22:11 --count 20 --interval 0.5

依赖:
  pip install bumble

目标设备 (07_phy_data_channel Demo):
  BLE Address: 66:55:44:33:22:11
  Device Name: DataCh
"""

import argparse
import asyncio
import logging
import struct
import sys
import time
from typing import Optional

from bumble.device import Device, Connection, ConnectionParametersPreferences
from bumble.hci import Address, Phy
from bumble.transport import open_transport


# 使用自定义 CID 避免与 bumble 内部 ATT/SMP handler 冲突
# Demo 回环不过滤 CID, 任何 CID 都会被 echo
ECHO_CID = 0x0040

# 连接断开原因码
DISCONNECT_REASONS = {
    0x08: "Connection Timeout",
    0x13: "Remote User Terminated",
    0x16: "Local Host Terminated",
    0x22: "LL Response Timeout",
    0x3E: "Failed to Establish",
}


def make_echo_payload(seq: int, size: int) -> bytes:
    """构造 echo 测试 payload: [seq_le32] [pattern...]"""
    header = struct.pack("<I", seq)  # 4 字节序列号
    if size <= 4:
        return header[:size]
    # 填充可识别的 pattern: 0xA0+i
    pattern = bytes((0xA0 + (i % 96)) for i in range(size - 4))
    return header + pattern


def verify_echo(sent: bytes, received: bytes) -> bool:
    """验证 echo payload 是否原样返回"""
    return sent == received


class L2capEchoTester:
    """L2CAP Echo 测试工具"""

    def __init__(
        self,
        transport: str,
        target: str,
        duration: int,
        count: int,
        size: int,
        interval: float,
        conn_interval: int,
        conn_timeout: int,
    ):
        self.transport_name = transport
        self.target_address = target.upper()
        self.duration = duration
        self.count = count
        self.payload_size = size
        self.send_interval = interval
        self.conn_interval = conn_interval
        self.conn_timeout = conn_timeout

        # 统计
        self.sent_count = 0
        self.recv_count = 0
        self.match_count = 0
        self.mismatch_count = 0
        self.sent_payloads: dict[int, tuple[bytes, float]] = {}  # seq -> (payload, send_time)
        self.latencies: list[float] = []

        self.connection: Optional[Connection] = None
        self._stop = asyncio.Event()
        self._echo_events: dict[int, asyncio.Event] = {}

    def _on_echo_received(self, connection_handle: int, pdu: bytes):
        """L2CAP 固定信道回调: 收到 echo 数据"""
        recv_time = time.monotonic()

        if len(pdu) < 4:
            print(f"  [ECHO] RX too short: {len(pdu)} bytes")
            return

        seq = struct.unpack("<I", pdu[:4])[0]
        self.recv_count += 1

        if seq in self.sent_payloads:
            sent_payload, send_time = self.sent_payloads[seq]
            latency_ms = (recv_time - send_time) * 1000

            if verify_echo(sent_payload, pdu):
                self.match_count += 1
                status = "OK"
            else:
                self.mismatch_count += 1
                status = "MISMATCH"
                print(f"  [ECHO] #{seq} {status}: sent {len(sent_payload)}B, got {len(pdu)}B")
                # 打印前 16 字节对比
                print(f"         sent: {sent_payload[:16].hex()}")
                print(f"         recv: {pdu[:16].hex()}")

            self.latencies.append(latency_ms)
            print(
                f"  [ECHO] #{seq} {status} | {len(pdu)}B | "
                f"latency={latency_ms:.1f}ms | "
                f"recv={self.recv_count}/{self.sent_count}"
            )

            # 通知等待者
            if seq in self._echo_events:
                self._echo_events[seq].set()
        else:
            print(f"  [ECHO] #{seq} unexpected (not in sent log)")

    def _on_disconnection(self, reason: int):
        reason_str = DISCONNECT_REASONS.get(reason, f"0x{reason:02X}")
        print(f"\n  ✖ DISCONNECTED: {reason_str}")
        self.connection = None
        self._stop.set()

    def _print_report(self):
        print(f"\n{'='*60}")
        print(f"  L2CAP ECHO TEST REPORT")
        print(f"{'='*60}")
        print(f"  Target       : {self.target_address}")
        print(f"  Payload size : {self.payload_size} bytes")
        ll_frags = (self.payload_size + 4 + 27 - 1) // 27  # +4 for L2CAP header
        print(f"  LL fragments : {ll_frags} per frame (MTU=27)")
        print()
        print(f"  Sent         : {self.sent_count}")
        print(f"  Received     : {self.recv_count}")
        print(f"  Match (OK)   : {self.match_count}")
        print(f"  Mismatch     : {self.mismatch_count}")
        lost = self.sent_count - self.recv_count
        print(f"  Lost         : {lost}")
        if self.sent_count > 0:
            success_pct = self.match_count / self.sent_count * 100
            print(f"  Success rate : {success_pct:.1f}%")
        if self.latencies:
            avg = sum(self.latencies) / len(self.latencies)
            print(f"\n  Latency (ms) :")
            print(f"    avg={avg:.1f}  min={min(self.latencies):.1f}  max={max(self.latencies):.1f}")
        print(f"{'='*60}\n")

    async def run(self):
        print(f"Bumble L2CAP Echo Tester")
        print(f"Transport    : {self.transport_name}")
        print(f"Target       : {self.target_address}")
        print(f"Payload size : {self.payload_size} bytes")
        print(f"Frame count  : {self.count}")
        print(f"Send interval: {self.send_interval}s")
        print()

        async with await open_transport(self.transport_name) as (
            hci_source,
            hci_sink,
        ):
            device = Device.with_hci(
                "EchoTester",
                Address("F0:F1:F2:F3:F4:F5"),
                hci_source,
                hci_sink,
            )

            # 注册 L2CAP 固定信道: 接收 echo 回传
            device.l2cap_channel_manager.register_fixed_channel(
                ECHO_CID, self._on_echo_received
            )

            await device.power_on()

            # ── 扫描目标设备 ──
            scan_done = asyncio.Event()
            found_adv = None

            def on_adv(adv):
                nonlocal found_adv
                if self.target_address in str(adv.address).upper():
                    found_adv = adv
                    scan_done.set()

            device.on("advertisement", on_adv)
            print(f"Scanning for {self.target_address}...")
            await device.start_scanning(active=False, filter_duplicates=True)

            try:
                await asyncio.wait_for(scan_done.wait(), timeout=10.0)
            except asyncio.TimeoutError:
                print("ERROR: Target not found within 10s")
                return
            await device.stop_scanning()
            device.remove_listener("advertisement", on_adv)

            # ── 建立连接 ──
            print(f"Connecting to {found_adv.address}...")
            try:
                interval_ms = self.conn_interval * 1.25
                timeout_ms = self.conn_timeout * 10
                conn_prefs = ConnectionParametersPreferences(
                    connection_interval_min=interval_ms,
                    connection_interval_max=interval_ms,
                    max_latency=0,
                    supervision_timeout=timeout_ms,
                )
                connection = await device.connect(
                    found_adv.address,
                    connection_parameters_preferences={Phy.LE_1M: conn_prefs},
                    timeout=10.0,
                )
            except Exception as exc:
                print(f"ERROR: Connection failed: {exc}")
                return

            self.connection = connection
            connection.on("disconnection", self._on_disconnection)

            print(f"  ★ Connected (handle=0x{connection.handle:04X})")
            print(f"  Waiting 1s for connection to stabilize...")
            await asyncio.sleep(1.0)

            if self._stop.is_set():
                self._print_report()
                return

            # ── 发送 L2CAP echo 帧 ──
            print(f"\n  Sending {self.count} L2CAP frames (CID=0x{ECHO_CID:04X})...\n")

            for seq in range(self.count):
                if self._stop.is_set():
                    break

                payload = make_echo_payload(seq, self.payload_size)
                self.sent_payloads[seq] = (payload, time.monotonic())
                self._echo_events[seq] = asyncio.Event()

                try:
                    device.l2cap_channel_manager.send_pdu(
                        connection, ECHO_CID, payload
                    )
                    self.sent_count += 1
                    print(
                        f"  [SEND] #{seq} | {len(payload)}B | "
                        f"head={payload[:8].hex()}"
                    )
                except Exception as exc:
                    print(f"  [SEND] #{seq} FAILED: {exc}")
                    break

                # 等待 echo 返回 (带超时)
                try:
                    await asyncio.wait_for(
                        self._echo_events[seq].wait(),
                        timeout=3.0,
                    )
                except asyncio.TimeoutError:
                    print(f"  [ECHO] #{seq} TIMEOUT (3s)")

                # 帧间间隔
                if seq < self.count - 1:
                    await asyncio.sleep(self.send_interval)

            # ── 等待最后的 echo ──
            if not self._stop.is_set():
                print(f"\n  Waiting 1s for remaining echoes...")
                await asyncio.sleep(1.0)

            # ── 主动断开连接 ──
            if self.connection is not None:
                print(f"  Disconnecting...")
                try:
                    await self.connection.disconnect()
                    # 等待 on_disconnection 回调
                    await asyncio.wait_for(self._stop.wait(), timeout=2.0)
                except (asyncio.TimeoutError, Exception):
                    pass

        self._print_report()


def main():
    parser = argparse.ArgumentParser(
        description="Bumble L2CAP Echo Tester — 测试 07_phy_data_channel 的 L2CAP 回环",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 基础 echo 测试 (5 帧, 16 字节)
  python bumble_l2cap_echo.py -t usb:2FE3:000B --target 66:55:44:33:22:11

  # 单 LL PDU 最大 payload (23 字节, L2CAP header 占 4 字节后剩 23 字节给 payload)
  python bumble_l2cap_echo.py -t usb:2FE3:000B --target 66:55:44:33:22:11 --size 23

  # 多分片测试 (40 字节 payload → L2CAP 44 字节 → 2 个 LL PDU)
  python bumble_l2cap_echo.py -t usb:2FE3:000B --target 66:55:44:33:22:11 --size 40

  # 大量快速发送 (20 帧, 间隔 200ms)
  python bumble_l2cap_echo.py -t usb:2FE3:000B --target 66:55:44:33:22:11 -n 20 --interval 0.2
        """,
    )

    parser.add_argument(
        "--transport", "-t", required=True,
        help="HCI transport (如: usb:2FE3:000B)",
    )
    parser.add_argument(
        "--target", required=True,
        help="目标设备 BLE 地址 (如: 66:55:44:33:22:11)",
    )
    parser.add_argument(
        "--count", "-n", type=int, default=5,
        help="发送帧数 (默认: 5)",
    )
    parser.add_argument(
        "--size", "-s", type=int, default=16,
        help="L2CAP payload 大小 (字节, 默认: 16, ≤23 单 PDU, >23 多分片)",
    )
    parser.add_argument(
        "--interval", "-i", type=float, default=1.0,
        help="帧间发送间隔 (秒, 默认: 1.0)",
    )
    parser.add_argument(
        "--conn-interval", type=int, default=40,
        help="连接 interval (1.25ms/unit, 默认: 40 = 50ms)",
    )
    parser.add_argument(
        "--conn-timeout", type=int, default=200,
        help="Supervision timeout (10ms/unit, 默认: 200 = 2s)",
    )
    parser.add_argument(
        "--duration", type=int, default=30,
        help="最大测试时长 (秒, 默认: 30)",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="启用 bumble 调试日志",
    )

    args = parser.parse_args()

    if args.size < 4:
        parser.error("--size must be >= 4 (need 4 bytes for sequence number)")
    if args.size > 512:
        parser.error("--size must be <= 512 (L2CAP_MAX_PAYLOAD)")

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    tester = L2capEchoTester(
        transport=args.transport,
        target=args.target,
        duration=args.duration,
        count=args.count,
        size=args.size,
        interval=args.interval,
        conn_interval=args.conn_interval,
        conn_timeout=args.conn_timeout,
    )

    asyncio.run(tester.run())


if __name__ == "__main__":
    main()
