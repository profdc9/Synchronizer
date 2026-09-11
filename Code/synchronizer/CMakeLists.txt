cmake_minimum_required(VERSION 3.13)

# The Pico W's onboard LED hangs off the wireless chip, and the whole
# project assumes wireless is present, so the board is not optional.
set(PICO_BOARD pico_w)

include(pico_sdk_import.cmake)

project(synchronizer C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

if (NOT PICO_CYW43_SUPPORTED)
    message(FATAL_ERROR "This project needs a Pico W; configure with -DPICO_BOARD=pico_w")
endif()

add_compile_options(
    -Wall
    -Wextra
    -Wno-format
    -Wno-unused-function
    -Wno-maybe-uninitialized
)

add_executable(synchronizer
    src/main.c
    src/config.c
    src/timebase.c
    src/sense.c
    src/drive.c
    src/control.c
    src/netclock.c
    src/cli.c
    src/httpd.c
    src/webui.c
    src/webpage.c
    src/tinycl.c
)

target_include_directories(synchronizer PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(synchronizer
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    hardware_pwm
    hardware_adc
    hardware_dma
    hardware_flash
    hardware_sync
    hardware_watchdog
)

pico_add_extra_outputs(synchronizer)

# The console is the USB CDC port; the UART pins are reserved for the GPS
# receiver that may be wired to J2 later.
pico_enable_stdio_usb(synchronizer 1)
pico_enable_stdio_uart(synchronizer 0)
