cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)

# Support for anonymous structs
add_definitions("-fms-extensions")

# include the SDK
include(../pico-sdk/pico_sdk_init.cmake)

project(supervisor C CXX)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

######

add_executable(bms
    bms.c
    model.c
    hw/internal_adc.c
    hw/duart.c
    hw/pwm.c
    hw/time.c
    hw/watchdog.c
    debug/counters.c
    isospi/isospi_master.c
    state_machines/base.c
    state_machines/contactors.c
    )

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
    hardware_adc
    hardware_dma
    hardware_pio
    hardware_pwm
)

set(PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 3000)

# enable usb output, disable uart output
pico_enable_stdio_usb(bms 1)
pico_enable_stdio_uart(bms 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(bms)
