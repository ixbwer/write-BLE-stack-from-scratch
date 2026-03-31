# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ble_phy_first_anchor)

target_sources(app PRIVATE
  src/main.c
  src/hal_radio.c
  src/ll_pdu.c
  src/adv.c
  src/conn.c
)

# Add BLE controller internal include paths so we can use HAL APIs and PDU structs
target_include_directories(app PRIVATE
  # controller root — resolves "hal/radio.h" -> controller/hal/radio.h (redirect)
  ${ZEPHYR_BASE}/subsys/bluetooth/controller
  # controller/include — resolves "ll.h" etc.
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/include
  # Nordic vendor — resolves "hal/radio_vendor_hal.h" -> "hal/nrf5/radio/radio.h"
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw/nordic
  # ll_sw — resolves "pdu.h", "pdu_df.h"
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw
  # Nordic LLL — resolves "pdu_vendor.h"
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw/nordic/lll
)
