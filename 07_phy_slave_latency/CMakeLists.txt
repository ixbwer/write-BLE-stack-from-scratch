# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ble_phy_slave_latency)

target_sources(app PRIVATE
  src/main.c
  ../stack/hal_radio.c
  ../stack/adv.c
  ../stack/conn.c
  ../stack/conn_event.c
  ../stack/anchor.c
  ../stack/slave_latency.c
  ../stack/ll_pdu.c
  ../stack/compat_globals.c
)

target_compile_definitions(app PRIVATE
  BLE_FEATURE_MULTI_PDU=0
  BLE_FEATURE_ATT=0
  BLE_FEATURE_SMP=0
  BLE_FEATURE_SLAVE_CPR=0
  BLE_FEATURE_SLAVE_TERMINATE=0
  BLE_FEATURE_GATT_NOTIFY=0
  BLE_FEATURE_LL_PROCEDURE=0
)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../stack
  ${ZEPHYR_BASE}/subsys/bluetooth/controller
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/include
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw/nordic
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw/nordic/lll
  ${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw/nordic/hal/nrf5/nrfx_glue
  ${ZEPHYR_BASE}/subsys/bluetooth
)
