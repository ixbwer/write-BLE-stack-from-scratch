# Write BLE Stack from Scratch

从零实现 BLE 协议栈 —— 基于 nRF52832 + Zephyr，逐层拆解 BLE 底层机制。

本仓库包含一系列递进式 Demo 和配套博客文章，从最底层的射频发包到完整的连接事件管理，
每一步都直接操作 nRF52 Radio HAL，不依赖现成的 BLE Host/Controller，帮助你理解「BLE 协议栈里面究竟发生了什么」。

## 环境要求

| 组件 | 说明 |
|------|------|
| **开发板** | nRF52 DK (nRF52832) |
| **SDK** | Zephyr RTOS (west 工具链) |
| **编译环境** | Zephyr Python venv (`D:\software\zephyrproject\.venv`) |
| **BLE 测试适配器** | nRF52840 Dongle (USB HCI)，或其他 Bumble 支持的 HCI 设备 |
| **Python 工具依赖** | [Bumble](https://github.com/nickelc/bumble) BLE 库（已安装在 `tools/venv/`） |
| **串口工具** | 任意串口终端 (115200, 8N1)，用于查看板级日志 |

## 快速开始

### 1. 编译

```powershell
# 激活 Zephyr 虚拟环境（自行搭建）

# 编译指定 Demo（-p 表示全量重建，切换不同 Demo 时需要加）
west build -b nrf52dk/nrf52832 .\write-BLE-stack-from-scratch\06_phy_anchor\ -p
```

### 2. 烧录

```powershell
west flash
```

### 3. 查看日志

用串口工具连接 nRF52 DK 的虚拟串口 (115200 baud)，观察运行日志。

### 4. 测试（需要 BLE USB 适配器）

```powershell
# 扫描广播包
.\write-BLE-stack-from-scratch\tools\venv\Scripts\python.exe ^
    .\write-BLE-stack-from-scratch\tools\bumble_scan_req.py ^
    --transport usb:2FE3:000B --target A6:55:44:33:22:11 --duration 10

# 建立连接 (interval=50ms, timeout=2s, 持续 15 秒)
.\write-BLE-stack-from-scratch\tools\venv\Scripts\python.exe ^
    .\write-BLE-stack-from-scratch\tools\bumble_connect.py ^
    --transport usb:2FE3:000B --target A6:55:44:33:22:11 --duration 15
```
