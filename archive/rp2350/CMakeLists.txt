cmake_minimum_required(VERSION 3.13)

set(QCOPTER_TARGET "flight" CACHE STRING "Firmware to configure: flight or controller")
set_property(CACHE QCOPTER_TARGET PROPERTY STRINGS flight controller)

if(NOT DEFINED PICO_SDK_PATH)
    if(DEFINED ENV{PICO_SDK_PATH})
        set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
    else()
        message(FATAL_ERROR "PICO_SDK_PATH is unset. Install the Pico SDK and set the environment variable.")
    endif()
endif()

if(NOT DEFINED PICO_BOARD)
    if(QCOPTER_TARGET STREQUAL "flight")
        set(PICO_BOARD pico2 CACHE STRING "RP2350 board; override for the final flight PCB")
    elseif(QCOPTER_TARGET STREQUAL "controller")
        set(PICO_BOARD pico CACHE STRING "RP2040 board; override for the selected controller board")
    else()
        message(FATAL_ERROR "QCOPTER_TARGET must be flight or controller")
    endif()
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(qcopter C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
pico_sdk_init()

if(QCOPTER_TARGET STREQUAL "flight")
    add_executable(qcopter
        flight_controller/flight_controller.c)
    target_link_libraries(qcopter pico_stdlib pico_multicore pico_util hardware_flash pico_flash pico_unique_id)
    # Reserve the last two erase sectors without changing the physical flash-size
    # macro used by the SDK flash driver. Link-time exclusion + runtime overlap check.
    math(EXPR QCOPTER_FIRMWARE_FLASH_BYTES "${PICO_FLASH_SIZE_BYTES} - 8192")
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/calibration-reserved")
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/calibration-reserved/pico_flash_region.ld"
        "FLASH(rx) : ORIGIN = 0x10000000, LENGTH = ${QCOPTER_FIRMWARE_FLASH_BYTES}\n")
    pico_add_linker_script_override_path(qcopter "${CMAKE_CURRENT_BINARY_DIR}/calibration-reserved" FILES pico_flash_region.ld)
elseif(QCOPTER_TARGET STREQUAL "controller")
    add_executable(qcopter handler_controller/handler_controller.c)
    target_link_libraries(qcopter pico_stdlib hardware_adc hardware_gpio)
else()
    message(FATAL_ERROR "QCOPTER_TARGET must be flight or controller")
endif()

target_compile_options(qcopter PRIVATE -Wall -Wextra -Werror)
pico_enable_stdio_usb(qcopter 1)
pico_enable_stdio_uart(qcopter 0)
pico_add_extra_outputs(qcopter)
