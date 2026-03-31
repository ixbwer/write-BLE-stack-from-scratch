#!/usr/bin/env python3
"""
Bumble BLE Active Scanner - 向指定地址设备发送 SCAN_REQ 并获取 SCAN_RSP

功能:
  1. 执行 BLE Active Scanning (主动扫描)
  2. 过滤指定目标地址的广播
  3. 收到 ADV_IND 后自动发送 SCAN_REQ
  4. 打印收到的 SCAN_RSP 数据

用法:
  # 使用 USB 连接的 HCI 适配器 (如 nRF52840 Dongle)
  python bumble_scan_req.py --transport usb:0 --target 66:55:44:33:22:11

  # 使用串口连接的 HCI 适配器
  python bumble_scan_req.py --transport serial:/dev/ttyACM0 --target 66:55:44:33:22:11

  # 使用 Bumble 虚拟控制器 (TCP)
  python bumble_scan_req.py --transport tcp-client:127.0.0.1:1234 --target 66:55:44:33:22:11

  # 扫描所有设备 (不过滤)
  python bumble_scan_req.py --transport usb:0

  # 指定扫描时长 (秒)
  python bumble_scan_req.py --transport usb:0 --target 66:55:44:33:22:11 --duration 30

依赖:
  pip install bumble
"""

import argparse
import asyncio
import logging
import struct
import sys
from typing import Optional

from bumble.core import AdvertisingData
from bumble.device import Device, Advertisement
from bumble.hci import Address
from bumble.transport import open_transport_or_link


# BLE AD Type 名称映射 (常见类型)
AD_TYPE_NAMES = {
    0x01: "Flags",
    0x02: "Incomplete List of 16-bit UUIDs",
    0x03: "Complete List of 16-bit UUIDs",
    0x04: "Incomplete List of 32-bit UUIDs",
    0x05: "Complete List of 32-bit UUIDs",
    0x06: "Incomplete List of 128-bit UUIDs",
    0x07: "Complete List of 128-bit UUIDs",
    0x08: "Shortened Local Name",
    0x09: "Complete Local Name",
    0x0A: "TX Power Level",
    0x0D: "Class of Device",
    0x0E: "Simple Pairing Hash C-192",
    0x0F: "Simple Pairing Randomizer R-192",
    0x10: "Device ID / Security Manager TK Value",
    0x11: "Security Manager OOB Flags",
    0x12: "Peripheral Connection Interval Range",
    0x14: "List of 16-bit Solicitation UUIDs",
    0x15: "List of 128-bit Solicitation UUIDs",
    0x16: "Service Data - 16-bit UUID",
    0x17: "Public Target Address",
    0x18: "Random Target Address",
    0x19: "Appearance",
    0x1A: "Advertising Interval",
    0x1B: "LE Bluetooth Device Address",
    0x1C: "LE Role",
    0x20: "Service Data - 32-bit UUID",
    0x21: "Service Data - 128-bit UUID",
    0xFF: "Manufacturer Specific Data",
}


def parse_ad_structures(data: bytes) -> list:
    """解析 AD (Advertising Data) 结构体列表"""
    structures = []
    offset = 0
    while offset < len(data):
        if offset + 1 > len(data):
            break
        length = data[offset]
        if length == 0:
            break
        if offset + 1 + length > len(data):
            break
        ad_type = data[offset + 1]
        ad_data = data[offset + 2 : offset + 1 + length]
        structures.append((ad_type, ad_data))
        offset += 1 + length
    return structures


def format_ad_structure(ad_type: int, ad_data: bytes) -> str:
    """格式化单个 AD 结构体为可读字符串"""
    type_name = AD_TYPE_NAMES.get(ad_type, f"Unknown(0x{ad_type:02X})")
    hex_str = ad_data.hex(" ")

    detail = ""
    if ad_type in (0x08, 0x09):  # Local Name
        try:
            detail = f' => "{ad_data.decode("utf-8")}"'
        except UnicodeDecodeError:
            pass
    elif ad_type == 0x01:  # Flags
        if len(ad_data) >= 1:
            flags = ad_data[0]
            parts = []
            if flags & 0x01:
                parts.append("LE Limited Discoverable")
            if flags & 0x02:
                parts.append("LE General Discoverable")
            if flags & 0x04:
                parts.append("BR/EDR Not Supported")
            if flags & 0x08:
                parts.append("LE+BR/EDR Controller")
            if flags & 0x10:
                parts.append("LE+BR/EDR Host")
            detail = f" => {' | '.join(parts)}" if parts else ""
    elif ad_type == 0x0A:  # TX Power Level
        if len(ad_data) >= 1:
            power = struct.unpack("b", ad_data)[0]
            detail = f" => {power} dBm"
    elif ad_type == 0xFF:  # Manufacturer Specific Data
        if len(ad_data) >= 2:
            company_id = struct.unpack("<H", ad_data[:2])[0]
            detail = f" => Company ID: 0x{company_id:04X}, Data: {ad_data[2:].hex(' ')}"

    return f"  [{type_name}] ({len(ad_data)} bytes): {hex_str}{detail}"


def format_address(address) -> str:
    """统一格式化地址"""
    return str(address)


class ScanReqTool:
    """BLE Active Scanner 工具类"""

    def __init__(self, transport: str, target: Optional[str], duration: int):
        self.transport_name = transport
        self.target_address = target.upper() if target else None
        self.duration = duration
        self.adv_count = 0
        self.scan_rsp_count = 0
        self.seen_addresses = set()

    def on_advertisement(self, advertisement: Advertisement):
        """广播/扫描响应回调"""
        addr_str = format_address(advertisement.address)

        # 如果指定了目标地址, 只处理目标设备
        if self.target_address and self.target_address not in addr_str.upper():
            return

        is_scan_rsp = advertisement.is_scan_response

        if is_scan_rsp:
            self.scan_rsp_count += 1
            print(f"\n{'='*60}")
            print(f"  ★ SCAN_RSP #{self.scan_rsp_count} from {addr_str}")
            print(f"{'='*60}")
        else:
            self.adv_count += 1
            adv_type = "ADV_IND" if advertisement.is_connectable else "ADV_NONCONN"
            print(f"\n{'─'*60}")
            print(f"  ADV #{self.adv_count}: {adv_type} from {addr_str}")
            print(f"{'─'*60}")

        # 打印 RSSI
        if advertisement.rssi is not None and advertisement.rssi != Advertisement.RSSI_NOT_AVAILABLE:
            print(f"  RSSI: {advertisement.rssi} dBm")

        # 打印 address type
        addr_type = advertisement.address.address_type
        addr_type_str = "Random" if addr_type else "Public"
        print(f"  Address Type: {addr_type_str}")

        # 解析并打印 AD 结构体
        raw_bytes = advertisement.data_bytes
        if raw_bytes:
            print(f"  Raw Data ({len(raw_bytes)} bytes): {raw_bytes.hex(' ')}")
            ad_structures = parse_ad_structures(raw_bytes)
            if ad_structures:
                print(f"  AD Structures ({len(ad_structures)}):")
                for ad_type, ad_data in ad_structures:
                    print(format_ad_structure(ad_type, ad_data))
        else:
            print("  (No AD data)")

        # 记录已发现的地址
        if addr_str not in self.seen_addresses:
            self.seen_addresses.add(addr_str)
            if not is_scan_rsp:
                print(f"  [NEW DEVICE]")

    async def run(self):
        print(f"Bumble BLE Active Scanner")
        print(f"Transport : {self.transport_name}")
        print(f"Target    : {self.target_address or 'ALL DEVICES'}")
        print(f"Duration  : {self.duration}s")
        print()

        # 打开 HCI transport
        print("Opening transport...")
        async with await open_transport_or_link(self.transport_name) as (
            hci_source,
            hci_sink,
        ):
            print("Transport opened, creating device...")

            # 创建 Bumble Device
            device = Device.with_hci(
                "BumbleScanner",
                Address("00:00:00:00:00:00"),  # 让控制器使用自己的地址
                hci_source,
                hci_sink,
            )

            # 注册广播回调
            device.on("advertisement", self.on_advertisement)

            # 上电
            await device.power_on()
            print(f"Device powered on, address: {device.public_address}")

            # 开始主动扫描 (Active Scanning)
            # Active Scanning 会在收到可扫描广播 (ADV_IND) 时自动发送 SCAN_REQ
            print(f"\nStarting Active Scan for {self.duration}s ...")
            print("(Active scan will automatically send SCAN_REQ for ADV_IND)\n")

            await device.start_scanning(
                active=True,                          # 主动扫描 → 发送 SCAN_REQ
                scan_interval=96,   # 60ms (96 * 0.625ms)
                scan_window=48,     # 30ms (48 * 0.625ms)
                filter_duplicates=False,              # 不过滤重复, 显示每次响应
            )

            # 等待扫描时长
            await asyncio.sleep(self.duration)

            # 停止扫描
            await device.stop_scanning()
            print(f"\n{'='*60}")
            print(f"Scan completed.")
            print(f"  ADV received     : {self.adv_count}")
            print(f"  SCAN_RSP received: {self.scan_rsp_count}")
            print(f"  Unique devices   : {len(self.seen_addresses)}")
            print(f"{'='*60}")


def main():
    parser = argparse.ArgumentParser(
        description="Bumble BLE Active Scanner - 发送 SCAN_REQ 并获取 SCAN_RSP",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Transport 示例:
  usb:0                          - 第一个 USB BLE 适配器
  usb:1                          - 第二个 USB BLE 适配器
  serial:COM3                    - Windows 串口
  serial:/dev/ttyACM0            - Linux 串口
  tcp-client:127.0.0.1:1234      - TCP 连接 (如 Bumble HCI bridge)
  android-netsim                 - Android 模拟器虚拟控制器

Address 格式:
  66:55:44:33:22:11              - 标准 BLE MAC 地址格式
        """,
    )

    parser.add_argument(
        "--transport",
        "-t",
        required=True,
        help="HCI transport (如: usb:0, serial:COM3, tcp-client:host:port)",
    )
    parser.add_argument(
        "--target",
        "-a",
        default=None,
        help="目标设备 BLE 地址 (如: 66:55:44:33:22:11), 不指定则扫描所有设备",
    )
    parser.add_argument(
        "--duration",
        "-d",
        type=int,
        default=10,
        help="扫描持续时间 (秒), 默认 10",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="启用详细日志",
    )

    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.WARNING)

    scanner = ScanReqTool(args.transport, args.target, args.duration)
    asyncio.run(scanner.run())


if __name__ == "__main__":
    main()
