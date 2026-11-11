cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)

# The Pico SDK's description of the chip package and flash, the same on every
# revision (not a BMS board: those are in boards/, selected with BMS_BOARD)
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
include(boards/boards.cmake)

set(BMS_BOARD "" CACHE STRING "Board to build for (see boards/README.md)")

add_executable(bms
    bms/app/bms.c
    bms/app/board_id.c
    bms/app/firmware_id.c
    bms/app/isolation.c
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
    bms/app/estimators/lifetime.c
    bms/app/estimators/fancy_count.c
    bms/app/estimators/ocv_learn.c
    bms/app/estimators/voltage_based.c
    bms/app/monitoring/counters.c
    bms/sys/events/events.c
    bms/sys/logging/logging.c
    bms/protocols/cli/cli.c
    bms/protocols/hmi_serial/hmi_serial.c
    bms/protocols/hmi_serial/hmi_registers.c
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
    bms/config/config_check.c
    bms/shared/cell_map.c
    bms/shared/supervisor_protocol.c
    bms/shared/duart_frame.c
    vendor/can2040/src/can2040.c
    vendor/littlefs/lfs.c
    vendor/littlefs/lfs_util.c
    main.c
    )

# The commit and time of this build (bms/app/firmware_id.c). Regenerated on every
# build so a rebuild after a commit or an edit is never stale.
set(BMS_FIRMWARE_ID_HEADER ${CMAKE_BINARY_DIR}/generated/firmware_id_generated.h)
add_custom_target(bms_firmware_id ALL
    COMMAND ${CMAKE_COMMAND} -DSRC=${CMAKE_CURRENT_LIST_DIR} -DOUT=${BMS_FIRMWARE_ID_HEADER}
        -P ${CMAKE_CURRENT_LIST_DIR}/cmake/firmware_id.cmake
    BYPRODUCTS ${BMS_FIRMWARE_ID_HEADER}
    COMMENT "Stamping the firmware ID"
)
add_dependencies(bms bms_firmware_id)
target_include_directories(bms PRIVATE ${CMAKE_BINARY_DIR}/generated)

pico_generate_pio_header(bms ${CMAKE_CURRENT_LIST_DIR}/vendor/can2040/pio/can2040.pio)

pico_generate_pio_header(bms ${CMAKE_CURRENT_LIST_DIR}/bms/drivers/isospi/isospi_master.pio)
pico_generate_pio_header(bms ${CMAKE_CURRENT_LIST_DIR}/bms/drivers/isospi/isosnoop.pio)
#pico_generate_pio_header(isosnooper ${CMAKE_CURRENT_LIST_DIR}/isospi_scope.pio)
#pico_generate_pio_header(isosnooper ${CMAKE_CURRENT_LIST_DIR}/isospi_device.pio)

# Disable C++ in this project
target_compile_definitions(bms PRIVATE PICO_CXX_DISABLE_ALLOCATION_OVERRIDES=1)
set_target_properties(bms PROPERTIES LINKER_LANGUAGE C)

set_source_files_properties(bms/config/config_check.c PROPERTIES COMPILE_OPTIONS -pedantic-errors)
bms_use_board(bms "${BMS_BOARD}")
message(STATUS "Board: ${BMS_BOARD}")
pico_set_program_description(bms "Pack profile: ${BMS_PROFILE}, board: ${BMS_BOARD}")

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
    m # lroundf: linking as C leaves libm out
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
