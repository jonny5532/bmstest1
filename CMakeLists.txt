cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)

set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/bms/config/boards)
set(PICO_BOARD bms_rp2354b)
set(PICO_PLATFORM rp2350)

# Limit the linker's flash region to 1.5MB to reserve space for NVM/LittleFS.
# Must be set before pico_sdk_init(), which generates the linker script.
# disabled as it causes writes to panic!
#set(PICO_FLASH_SIZE_BYTES 1572864)

# Support for anonymous structs
add_definitions("-fms-extensions")

# Locate the SDK via PICO_SDK_PATH (environment or -D), must be before project()
include(pico_sdk_import.cmake)

project(bms C CXX ASM)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

######

include_directories(bms vendor/can2040/src)

include(profiles/profiles.cmake)

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
    bms/drivers/chip/nvm.c
    bms/drivers/chip/i2c.c
    bms/drivers/chip/pwm.c
    bms/sys/time/time.c
    bms/drivers/chip/watchdog.c
    bms/drivers/bmb3y/bmb3y.c
    bms/drivers/bmb3y/crc.c
    bms/app/battery/balancing.c
    bms/app/battery/current_limits.c
    bms/app/battery/safety_checks.c
    bms/app/calibration/common.c
    bms/app/calibration/calibration.c
    bms/app/estimators/basic_count.c
    bms/app/estimators/ekf.c
    bms/app/estimators/fancy_count.c
    bms/app/estimators/ocv_learn.c
    bms/app/estimators/voltage_based.c
    bms/app/monitoring/counters.c
    bms/sys/events/events.c
    bms/sys/logging/logging.c
    bms/protocols/cli/cli.c
    bms/protocols/hmi_serial/hmi_serial.c
    bms/protocols/hmi_serial/hmi_arrays.c
    bms/protocols/internal_serial/internal_serial.c
    bms/protocols/inverter/can.c
    bms/protocols/inverter/byd_can.c
    bms/drivers/isospi/isospi_master.c
    bms/drivers/isospi/isospi_master_async.c
    bms/drivers/isospi/isosnoop.c
    bms/app/state_machines/base.c
    bms/app/state_machines/contactors.c
    bms/app/state_machines/system.c
    bms/lib/sampler.c
    bms/config/profile_check.c
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

# Disable C++ in this project
target_compile_definitions(bms PRIVATE PICO_CXX_DISABLE_ALLOCATION_OVERRIDES=1)
set_target_properties(bms PROPERTIES LINKER_LANGUAGE C)

set_source_files_properties(bms/config/profile_check.c PROPERTIES COMPILE_OPTIONS -pedantic-errors)
pico_set_program_description(bms "Pack profile: ${BMS_PROFILE}")

if(BMS_BOARD)
    add_compile_definitions(BMS_BOARD=${BMS_BOARD})
endif()

# Boards with USB VBUS wired to GPIO3 (J12 pin 2) through a 3.3k/5.1k divider.
# The chip's D+ pull-up is then only switched on while the host supplies VBUS,
# so a switched-off host is never back-powered through D+. Leave OFF on boards
# without the divider, or USB never connects.
option(BMS_USB_VBUS_SENSE "Board has USB VBUS sensed on GPIO3" OFF)
if(BMS_USB_VBUS_SENSE)
    add_compile_definitions(BMS_USB_VBUS_SENSE=1)
endif()

# Passive isoSPI listener that dumps the raw bus symbols when a BMB CRC fails.
# Bench debugging only: it uses all of one PIO, a DMA channel and GPIO30.
option(BMS_ISOSNOOP "Enable the isoSPI snooper debug dump" OFF)
if(BMS_ISOSNOOP)
    add_compile_definitions(BMS_ISOSNOOP=1)
endif()

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

# Third-party code: don't apply our stricter warnings
set_source_files_properties(
    vendor/can2040/src/can2040.c
    vendor/littlefs/lfs.c
    vendor/littlefs/lfs_util.c
    PROPERTIES COMPILE_OPTIONS "-Wno-unused-parameter;-Wno-cast-qual;-Wno-null-dereference"
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
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=3000
)

# enable usb output, disable uart output
pico_enable_stdio_usb(bms 1)
pico_enable_stdio_uart(bms 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(bms)
