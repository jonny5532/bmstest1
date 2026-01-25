cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)

set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/bms/config/boards)
set(PICO_BOARD bms_rp2354b)

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

include_directories(bms vendor/can2040/src)

add_executable(bms
    bms/app/bms.c
    bms/app/hardware_checks.c
    bms/app/init.c
    bms/app/model.c
    bms/drivers/contactors/contactors.c
    bms/drivers/sensors/internal_adc.c
    bms/drivers/sensors/ina228.c
    bms/drivers/sensors/ads1115.c
    bms/drivers/comms/duart.c
    bms/protocols/hmi_serial/hmi_serial.c
    bms/protocols/internal_serial/internal_serial.c
    bms/drivers/chip/nvm.c
    bms/drivers/chip/pwm.c
    bms/sys/time/time.c
    bms/drivers/chip/watchdog.c
    bms/drivers/bmb3y/bmb3y.c
    bms/drivers/bmb3y/crc.c
    bms/app/battery/balancing.c
    bms/app/battery/current_limits.c
    bms/app/battery/safety_checks.c
    bms/app/calibration/offline.c
    bms/app/estimators/basic_count.c
    bms/app/estimators/ekf.c
    bms/app/estimators/fancy_count.c
    bms/app/estimators/voltage_based.c
    bms/app/monitoring/counters.c
    bms/sys/events/events.c
    bms/protocols/inverter/byd_can.c
    bms/drivers/isospi/isospi_master.c
    bms/drivers/isospi/isosnoop.c
    bms/app/state_machines/base.c
    bms/app/state_machines/contactors.c
    bms/app/state_machines/system.c
    bms/lib/sampler.c
    vendor/can2040/src/can2040.c
    vendor/littlefs/lfs.c
    vendor/littlefs/lfs_util.c
    main.c
    )

pico_generate_pio_header(bms ${CMAKE_CURRENT_LIST_DIR}/vendor/can2040/pio/can2040.pio)

pico_generate_pio_header(bms ${CMAKE_CURRENT_LIST_DIR}/bms/drivers/isospi/isospi_master.pio)
pico_generate_pio_header(bms ${CMAKE_CURRENT_LIST_DIR}/bms/drivers/isospi/isosnoop.pio)
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
    pico_float
    hardware_adc
    hardware_dma
    hardware_flash
    hardware_i2c
    hardware_pio
    hardware_pwm
    cmsis_core
)

target_compile_definitions(bms PRIVATE
    USBD_MANUFACTURER="S3XY-BMS"
    USBD_PRODUCT="Controller"
)

set(PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 3000)

# enable usb output, disable uart output
pico_enable_stdio_usb(bms 1)
pico_enable_stdio_uart(bms 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(bms)
