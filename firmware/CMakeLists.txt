cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ble_triangulation_receiver)

target_sources(app PRIVATE
    src/main.c
    src/ble/ble_manager.c
    src/ble/cs_adapter.c
    src/ble/cs_estimator.c
    src/ble/cs_phase_unwrap.c
    src/triangulation/triangulation.c
    src/triangulation/rssi_filter.c
)

target_include_directories(app PRIVATE
    src
)
