#!/usr/bin/env python3
"""
Bumble BLE Connection Tester - 向指定地址设备发起 BLE 连接并测试连接质量

功能:
  1. 扫描并发现目标 BLE 广播设备
  2. 发送 CONNECT_IND 建立 BLE 连接 (Central/Master 角色)
  3. 监控连接建立、参数、数据交换及断开事件
  4. 打印连接参数 (interval, latency, timeout, channel map)
  5. 定期读取 RSSI 并统计连接质量
  6. 执行 LL 控制过程: Version Exchange, Feature Exchange
  7. 连接结束后输出完整统计报告

用法:
  # 连接指定地址的 Anchor Demo (nRF52 DK)
  python bumble_connect.py --transport usb:2FE3:000B --target 66:55:44:33:22:11

  # 指定连接参数 (interval=100ms, latency=0, timeout=2000ms)
  python bumble_connect.py --transport usb:2FE3:000B --target 66:55:44:33:22:11 \\
      --conn-interval 80 --conn-latency 0 --conn-timeout 200

  # 连接后保持 30 秒
  python bumble_connect.py --transport usb:2FE3:000B --target 66:55:44:33:22:11 --duration 30

  # 连接后立即请求更新连接参数
  python bumble_connect.py --transport usb:2FE3:000B --target 66:55:44:33:22:11 \\
      --update-interval 24 --update-latency 2 --update-timeout 400

  # 使用串口适配器
  python bumble_connect.py --transport serial:COM3 --target 66:55:44:33:22:11

依赖:
  pip install bumble

目标设备 (06_phy_anchor Demo):
  BLE Address: 66:55:44:33:22:11
  Device Name: Anchor
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


# 连接断开原因码映射 (HCI Error Codes)
DISCONNECT_REASONS = {
    0x00: "Success / Normal",
    0x02: "Unknown Connection Identifier",
    0x05: "Authentication Failure",
    0x06: "PIN or Key Missing",
    0x07: "Memory Capacity Exceeded",
    0x08: "Connection Timeout",
    0x09: "Connection Limit Exceeded",
    0x0B: "ACL Connection Already Exists",
    0x0C: "Command Disallowed",
    0x13: "Remote User Terminated Connection",
    0x14: "Remote Host Terminated - Low Resources",
    0x15: "Remote Host Terminated - Power Off",
    0x16: "Connection Terminated by Local Host",
    0x22: "LL Response Timeout",
    0x28: "LL Instant Passed",
    0x29: "Pairing with Unit Key Not Supported",
    0x3A: "Controller Busy",
    0x3B: "Unacceptable Connection Parameters",
    0x3C: "Advertising Timeout",
    0x3D: "Connection Terminated due to MIC Failure",
    0x3E: "Failed to Establish Connection",
}


def decode_disconnect_reason(reason: int) -> str:
    return DISCONNECT_REASONS.get(reason, f"Unknown (0x{reason:02X})")


class BleConnectionTester:
    """BLE 连接测试工具类"""

    def __init__(
        self,
        transport: str,
        target: Optional[str],
        duration: int,
        conn_interval: int,
        conn_latency: int,
        conn_timeout: int,
        update_interval: Optional[int],
        update_latency: Optional[int],
        update_timeout: Optional[int],
        rssi_interval: int,
    ):
        self.transport_name = transport
        self.target_address = target.upper() if target else None
        self.duration = duration

        # 初始连接参数 (单位: 1.25ms for interval, 10ms for timeout)
        self.conn_interval = conn_interval      # 1.25ms units
        self.conn_latency = conn_latency
        self.conn_timeout = conn_timeout        # 10ms units

        # 参数更新请求 (可选)
        self.update_interval = update_interval
        self.update_latency = update_latency
        self.update_timeout = update_timeout
        self.rssi_interval = rssi_interval

        # 统计
        self.connect_start_time: Optional[float] = None
        self.connect_count = 0
        self.disconnect_count = 0
        self.rssi_samples: list = []
        self.param_updates: list = []
        self.connection: Optional[Connection] = None

        self._stop_event = asyncio.Event()

    def _print_separator(self, char="─", width=60):
        print(char * width)

    def on_connection(self, connection: Connection):
        """连接建立回调"""
        self.connect_start_time = time.time()
        self.connect_count += 1
        self.connection = connection

        params = connection.parameters

        print(f"\n{'='*60}")
        print(f"  ★ BLE CONNECTION ESTABLISHED #{self.connect_count}")
        print(f"{'='*60}")
        print(f"  Peer Address : {connection.peer_address}")
        print(f"  Handle       : 0x{connection.handle:04X}")
        role_str = "Central (Master)" if connection.role == 0 else "Peripheral (Slave)"
        print(f"  Role         : {role_str}")

        if params:
            interval_us = params.connection_interval * 1250
            timeout_ms = params.supervision_timeout * 10
            print(f"\n  ── Connection Parameters ──")
            print(
                f"  Interval     : {params.connection_interval} units"
                f" = {interval_us}μs = {interval_us / 1000:.2f}ms"
            )
            print(f"  Latency      : {params.peripheral_latency} events")
            print(
                f"  Sup.Timeout  : {params.supervision_timeout} units"
                f" = {timeout_ms}ms"
            )

        print(f"{'='*60}\n")

        # 注册参数更新回调
        connection.on("connection_parameters_update", self.on_params_update)
        connection.on("disconnection", self.on_disconnection)

    def on_disconnection(self, reason: int):
        """连接断开回调"""
        elapsed = (
            time.time() - self.connect_start_time
            if self.connect_start_time
            else 0.0
        )
        self.disconnect_count += 1
        reason_str = decode_disconnect_reason(reason)

        print(f"\n{'!'*60}")
        print(f"  ✖ DISCONNECTED")
        print(f"  Reason  : 0x{reason:02X} - {reason_str}")
        print(f"  Duration: {elapsed:.2f}s")
        if self.rssi_samples:
            avg_rssi = sum(self.rssi_samples) / len(self.rssi_samples)
            print(f"  RSSI    : avg={avg_rssi:.1f}dBm  min={min(self.rssi_samples)}dBm  max={max(self.rssi_samples)}dBm  samples={len(self.rssi_samples)}")
        print(f"{'!'*60}\n")

        self.connect_start_time = None
        self.connection = None
        self._stop_event.set()

    def on_params_update(self, params):
        """连接参数更新回调"""
        interval_us = params.connection_interval * 1250
        timeout_ms = params.supervision_timeout * 10
        self.param_updates.append({
            "time": time.time(),
            "interval": params.connection_interval,
            "latency": params.peripheral_latency,
            "timeout": params.supervision_timeout,
        })

        print(f"\n  ◆ Connection Parameters Updated:")
        print(
            f"    Interval : {params.connection_interval} units"
            f" = {interval_us}μs = {interval_us / 1000:.2f}ms"
        )
        print(f"    Latency  : {params.peripheral_latency} events")
        print(f"    Timeout  : {params.supervision_timeout} units = {timeout_ms}ms\n")

    async def _periodically_read_rssi(self, connection: Connection):
        """定期读取 RSSI"""
        while not self._stop_event.is_set() and self.connection is not None:
            try:
                rssi = await connection.get_rssi()
                self.rssi_samples.append(rssi)
                elapsed = (
                    time.time() - self.connect_start_time
                    if self.connect_start_time
                    else 0.0
                )
                print(f"  [t={elapsed:6.1f}s] RSSI = {rssi:+4d} dBm")
            except Exception as exc:
                print(f"  [RSSI] read failed: {exc}")
                break
            await asyncio.sleep(self.rssi_interval)

    async def _do_ll_procedures(self, connection: Connection):
        """执行 LL 控制过程: Feature 交换"""
        await asyncio.sleep(0.3)  # 等待连接稳定

        # LE Feature Exchange
        try:
            print("  ── LL Feature Exchange ──")
            features = await connection.get_remote_le_features()
            print(f"  Remote LE Features: {features}")
        except Exception as exc:
            print(f"  [Feature Exchange] failed: {exc}")

        # 如果指定了参数更新, 发起请求
        if self.update_interval is not None:
            await asyncio.sleep(0.5)
            try:
                print(f"  ── Request Connection Parameters Update ──")
                print(
                    f"  Requesting: interval={self.update_interval}"
                    f"  latency={self.update_latency}"
                    f"  timeout={self.update_timeout}"
                )
                await connection.update_parameters(
                    connection_interval_min=self.update_interval * 1.25,
                    connection_interval_max=self.update_interval * 1.25,
                    max_latency=self.update_latency if self.update_latency else 0,
                    supervision_timeout=(self.update_timeout if self.update_timeout else self.conn_timeout) * 10.0,
                )
            except Exception as exc:
                print(f"  [Params Update] failed: {exc}")

    def _print_final_report(self):
        """打印最终统计报告"""
        print(f"\n{'='*60}")
        print(f"  CONNECTION TEST REPORT")
        print(f"{'='*60}")
        print(f"  Target         : {self.target_address or 'ANY'}")
        print(f"  Connections    : {self.connect_count}")
        print(f"  Disconnections : {self.disconnect_count}")

        if self.rssi_samples:
            avg_rssi = sum(self.rssi_samples) / len(self.rssi_samples)
            print(f"\n  RSSI Statistics:")
            print(f"    Samples : {len(self.rssi_samples)}")
            print(f"    Average : {avg_rssi:.1f} dBm")
            print(f"    Min     : {min(self.rssi_samples)} dBm")
            print(f"    Max     : {max(self.rssi_samples)} dBm")

        if self.param_updates:
            print(f"\n  Parameter Updates: {len(self.param_updates)}")
            for i, upd in enumerate(self.param_updates):
                iu = upd["interval"] * 1250
                print(
                    f"    [{i+1}] interval={upd['interval']}({iu / 1000:.2f}ms)"
                    f"  latency={upd['latency']}"
                    f"  timeout={upd['timeout'] * 10}ms"
                )

        print(f"{'='*60}\n")

    async def run(self):
        print(f"Bumble BLE Connection Tester")
        print(f"Transport  : {self.transport_name}")
        print(f"Target     : {self.target_address or 'ANY (first found)'}")
        interval_ms = self.conn_interval * 1.25
        print(f"Interval   : {self.conn_interval} units = {interval_ms:.2f}ms")
        print(f"Latency    : {self.conn_latency}")
        print(f"Timeout    : {self.conn_timeout} units = {self.conn_timeout * 10}ms")
        print(f"Duration   : {self.duration}s")
        print()

        print("Opening transport...")
        async with await open_transport(self.transport_name) as (
            hci_source,
            hci_sink,
        ):
            print("Transport opened, creating device...")

            device = Device.with_hci(
                "BumbleConnector",
                Address("F0:F1:F2:F3:F4:F5"),
                hci_source,
                hci_sink,
            )

            # 注册连接回调
            device.on("connection", self.on_connection)

            await device.power_on()
            print(f"Device powered on, address: {device.public_address}")

            # 先扫描找到目标设备
            scan_done = asyncio.Event()
            found_adv = None

            def on_adv(advertisement):
                nonlocal found_adv
                addr_str = str(advertisement.address).upper()
                if self.target_address:
                    if self.target_address not in addr_str:
                        return
                print(f"  Found target: {advertisement.address}"
                      f"  RSSI={advertisement.rssi}dBm")
                found_adv = advertisement
                scan_done.set()

            device.on("advertisement", on_adv)

            print(f"Scanning for target {self.target_address or 'ANY'}...")
            await device.start_scanning(active=False, filter_duplicates=True)

            try:
                await asyncio.wait_for(scan_done.wait(), timeout=10.0)
            except asyncio.TimeoutError:
                print("ERROR: Target device not found within 10s.")
                print("  Check that the nRF52 DK is flashed and advertising.")
                await device.stop_scanning()
                return

            await device.stop_scanning()
            device.remove_listener("advertisement", on_adv)

            # 发起连接
            peer_address = found_adv.address
            print(f"\nConnecting to {peer_address}...")
            print(f"  Sending CONNECT_IND with:")
            print(f"    interval = {self.conn_interval} ({self.conn_interval * 1.25:.2f}ms)")
            print(f"    latency  = {self.conn_latency}")
            print(f"    timeout  = {self.conn_timeout} ({self.conn_timeout * 10}ms)")

            try:
                # Bumble API: interval 单位是 ms, supervision_timeout 单位是 ms
                # CLI --conn-timeout 使用 10ms/unit, 需要 ×10 转为 ms
                interval_ms = self.conn_interval * 1.25
                timeout_ms = self.conn_timeout * 10
                conn_prefs = ConnectionParametersPreferences(
                    connection_interval_min=interval_ms,
                    connection_interval_max=interval_ms,
                    max_latency=self.conn_latency,
                    supervision_timeout=timeout_ms,
                )
                connection = await device.connect(
                    peer_address,
                    connection_parameters_preferences={Phy.LE_1M: conn_prefs},
                    timeout=10.0,
                )
            except Exception as exc:
                print(f"ERROR: Connection failed: {exc}")
                self._print_final_report()
                return

            # 启动并发任务: RSSI 读取 + LL 控制过程
            rssi_task = asyncio.create_task(
                self._periodically_read_rssi(connection)
            )
            ll_task = asyncio.create_task(
                self._do_ll_procedures(connection)
            )

            # 等待: duration 超时 或 连接断开
            try:
                await asyncio.wait_for(
                    self._stop_event.wait(),
                    timeout=float(self.duration),
                )
            except asyncio.TimeoutError:
                print(f"\n  Duration {self.duration}s elapsed, disconnecting...")

            # 取消后台任务
            rssi_task.cancel()
            ll_task.cancel()
            try:
                await rssi_task
            except asyncio.CancelledError:
                pass
            try:
                await ll_task
            except asyncio.CancelledError:
                pass

            # 主动断开 (如果还连着)
            if self.connection is not None:
                try:
                    await self.connection.disconnect()
                    # 等待 on_disconnection 回调触发
                    await asyncio.sleep(0.5)
                except Exception as exc:
                    print(f"  [Disconnect] {exc}")

        self._print_final_report()


def main():
    parser = argparse.ArgumentParser(
        description="Bumble BLE Connection Tester - 发起 CONNECT_IND 并测试连接质量",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Transport 示例:
  usb:2FE3:000B                  - 按 VID:PID 指定 USB HCI 适配器 (如 nRF52840 Dongle)
  usb:0                          - 第一个 USB BLE 适配器
  serial:COM3                    - Windows 串口
  serial:/dev/ttyACM0            - Linux 串口
  tcp-client:127.0.0.1:1234      - TCP 连接

连接参数单位:
  --conn-interval   : 1.25ms/unit,  范围 6~3200  (7.5ms ~ 4s),   默认 40 (50ms)
  --conn-latency    : 事件数,        范围 0~499,                   默认 0
  --conn-timeout    : 10ms/unit,    范围 10~3200 (100ms ~ 32s),   默认 200 (2s)

典型用法:
  # 测试 06_phy_anchor Demo
  python bumble_connect.py --transport usb:2FE3:000B --target 66:55:44:33:22:11

  # 快速连接测试 (小 interval, 短 timeout)
  python bumble_connect.py --transport usb:0 --target 66:55:44:33:22:11 \\
      --conn-interval 8 --conn-timeout 100 --duration 10

  # 测试参数更新
  python bumble_connect.py --transport usb:0 --target 66:55:44:33:22:11 \\
      --conn-interval 40 --update-interval 16 --update-latency 4 --update-timeout 400
        """,
    )

    parser.add_argument(
        "--transport", "-t",
        required=True,
        help="HCI transport (如: usb:0, usb:VID:PID, serial:COM3)",
    )
    parser.add_argument(
        "--target", "-a",
        default=None,
        help="目标设备 BLE 地址 (如: 66:55:44:33:22:11), 不指定则连接第一个发现的设备",
    )
    parser.add_argument(
        "--duration", "-d",
        type=int,
        default=20,
        help="连接保持时间 (秒), 默认 20",
    )
    parser.add_argument(
        "--conn-interval",
        type=int,
        default=40,
        help="连接 interval (1.25ms/unit), 默认 40 (=50ms)",
    )
    parser.add_argument(
        "--conn-latency",
        type=int,
        default=0,
        help="连接 peripheral latency (事件数), 默认 0",
    )
    parser.add_argument(
        "--conn-timeout",
        type=int,
        default=200,
        help="连接 supervision timeout (10ms/unit), 默认 200 (=2s)",
    )
    parser.add_argument(
        "--update-interval",
        type=int,
        default=None,
        help="连接后请求更新 interval (1.25ms/unit), 不指定则不发更新请求",
    )
    parser.add_argument(
        "--update-latency",
        type=int,
        default=0,
        help="更新后的 peripheral latency, 默认 0",
    )
    parser.add_argument(
        "--update-timeout",
        type=int,
        default=None,
        help="更新后的 supervision timeout (10ms/unit)",
    )
    parser.add_argument(
        "--rssi-interval",
        type=int,
        default=3,
        help="RSSI 读取间隔 (秒), 默认 3",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="启用详细 HCI 日志",
    )

    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.WARNING)

    tester = BleConnectionTester(
        transport=args.transport,
        target=args.target,
        duration=args.duration,
        conn_interval=args.conn_interval,
        conn_latency=args.conn_latency,
        conn_timeout=args.conn_timeout,
        update_interval=args.update_interval,
        update_latency=args.update_latency,
        update_timeout=args.update_timeout,
        rssi_interval=args.rssi_interval,
    )
    asyncio.run(tester.run())


if __name__ == "__main__":
    main()
