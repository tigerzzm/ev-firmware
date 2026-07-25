cmake_minimum_required(VERSION 3.13)

# ==== Raspberry Pi Pico VS Code Extension block (safe to keep) ================
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# =============================================================================

# Target board: Pico 2 (RP2350). (robotour used "pico" / RP2040.)
set(PICO_BOARD pico2 CACHE STRING "Board type")

set(PICO_SDK_FETCH_FROM_GIT on)
include(pico_sdk_import.cmake)

project(ev_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

file(GLOB_RECURSE SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/*.c
  ${CMAKE_CURRENT_LIST_DIR}/src/BNO08x/*.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/BNO08x/*.c
)

add_executable(ev_firmware ${SOURCES})

target_include_directories(ev_firmware PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

# Generate the quadrature encoder PIO header.
pico_generate_pio_header(ev_firmware
  ${CMAKE_CURRENT_LIST_DIR}/src/quadrature_encoder.pio)

pico_enable_stdio_usb(ev_firmware 1)
pico_enable_stdio_uart(ev_firmware 0)

pico_add_extra_outputs(ev_firmware)

target_link_libraries(ev_firmware
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_i2c
  hardware_pwm
  hardware_adc
)
