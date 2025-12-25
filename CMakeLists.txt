cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)

# Support for anonymous structs
add_definitions("-fms-extensions")

# include the SDK
include(../pico-sdk/pico_sdk_init.cmake)

project(supervisor C CXX)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Limit flash size to 1.5MB to reserve space for NVM/LittleFS
set(PICO_FLASH_SIZE_BYTES 1572864)

######

include_directories(vendor/can2040/src)

add_executable(bms
    bms.c
    model.c
    hw/contactors.c
    hw/internal_adc.c
    hw/ina228.c
    hw/ads1115.c
    hw/duart.c
    hw/nvm.c
    hw/pwm.c
    hw/time.c
    hw/watchdog.c
    bmb3y/bmb3y.c
    battery/balancing.c
    debug/counters.c
    debug/events.c
    inverter/byd_can.c
    isospi/isospi_master.c
    state_machines/base.c
    state_machines/contactors.c
    util/sampler.c
    vendor/can2040/src/can2040.c
    vendor/littlefs/lfs.c
    vendor/littlefs/lfs_util.c
    )

pico_generate_pio_header(bms ${CMAKE_CURRENT_LIST_DIR}/vendor/can2040/pio/can2040.pio)


pico_generate_pio_header(bms ${CMAKE_CURRENT_LIST_DIR}/isospi/isospi_master.pio)
#pico_generate_pio_header(isosnooper ${CMAKE_CURRENT_LIST_DIR}/isosnoop.pio)
#pico_generate_pio_header(isosnooper ${CMAKE_CURRENT_LIST_DIR}/isospi_scope.pio)
#pico_generate_pio_header(isosnooper ${CMAKE_CURRENT_LIST_DIR}/isospi_device.pio)

target_compile_options(bms PUBLIC
    -Wall
    -Wextra
    -Wnull-dereference
    -Wuninitialized
    #-Wunused
    -Wno-unused-function
    -Wno-unused-variable
    -Wcast-align
    -Wcast-qual
    -Wfloat-equal
    -Wmissing-format-attribute
)


# pull in common dependencies
target_link_libraries(bms 
    pico_stdlib
    pico_multicore
    pico_flash
    hardware_adc
    hardware_dma
    hardware_flash
    hardware_i2c
    hardware_pio
    hardware_pwm
)

set(PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 3000)

# enable usb output, disable uart output
pico_enable_stdio_usb(bms 1)
pico_enable_stdio_uart(bms 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(bms)
