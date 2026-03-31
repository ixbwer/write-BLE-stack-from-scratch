# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ble_phy_data_channel)

target_sources(app PRIVATE
  src/main.c
  src/hal_radio.c
  src/ll_pdu.c
  src/adv.c
  src/conn.c
  src/l2cap.c
)

# Add BLE controller internal include paths so we can use HAL APIs and PDU structs
target_include_directories(app PRIVATE
  ${ZEPHYR_BASE}/subsys/bluetooth/controller
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/include
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw/nordic
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw/nordic/lll
)
