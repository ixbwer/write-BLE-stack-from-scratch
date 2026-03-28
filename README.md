# Write BLE Stack from Scratch

从零实现 BLE 协议栈 —— 基于 nRF52832 + Zephyr，逐层拆解 BLE 底层机制。

本仓库包含一系列递进式 Demo 和配套博客文章，从最底层的射频发包到完整的连接事件管理，
帮助你理解「BLE 协议栈里面究竟发生了什么」。

## 环境要求

| 组件 | 说明 |
|------|------|
| **开发板** | nRF52 DK (nRF52832) |
| **SDK** | Zephyr RTOS (west 工具链) |
| **编译环境** | Zephyr Python venv |

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

## 系列教程
- [【从零开始】手写BLE协议栈（0-1）开篇与硬件选型](https://www.cnblogs.com/ixbwer/p/19780996)
- [【从零开始】手写BLE协议栈（1-1）NRF 52 RADIO 相关机制](https://www.cnblogs.com/ixbwer/p/19781637)
- [【从零开始】手写BLE协议栈（1-2）Hello World：BLE广播](https://www.cnblogs.com/ixbwer/p/19781641)
- [【从零开始】手写BLE协议栈（2-1）主动扫描（Active Scanning）：三包握手](https://www.cnblogs.com/ixbwer/p/19786057)
- [【从零开始】手写BLE协议栈（2-2）T_IFS：为什么是150 µs](https://www.cnblogs.com/ixbwer/p/19786207)