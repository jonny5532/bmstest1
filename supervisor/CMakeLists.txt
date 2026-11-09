cmake_minimum_required(VERSION 3.13)

# Supervisor MCU firmware: an RP2350 alongside the BMS MCU that holds the
# contactor gate drivers disabled (CTR_EN) unless its own checks of the cell
# voltages it snoops off the isoSPI bus pass. See README.md ("Supervisor").
#
#   cmake -S supervisor -B supervisor/build -DSUPERVISOR_CHEMISTRY=lfp -DBMS_BOARD=desk
#
# It is built per chemistry, not per pack: the cell map comes from the BMS.

set(CMAKE_C_STANDARD 11)

# The same chip as the BMS MCU (an RP2354B: it uses GPIOs above 29)
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/../bms/config/boards)
set(PICO_BOARD bms_rp2354b)
set(PICO_PLATFORM rp2350)

# Locate the SDK via PICO_SDK_PATH (environment or -D), must be before project()
include(${CMAKE_CURRENT_LIST_DIR}/../pico_sdk_import.cmake)

project(supervisor C CXX ASM)

pico_sdk_init()

######

set(BMS_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
set(BMS_DIR ${BMS_ROOT}/bms)

# The chemistry sets the hard cell voltage limits the supervisor enforces
set(SUPERVISOR_CHEMISTRY "" CACHE STRING "Pack chemistry the supervisor enforces: lfp or nmc")
if(NOT SUPERVISOR_CHEMISTRY MATCHES "^(lfp|nmc)$")
    message(FATAL_ERROR "Set the chemistry with -DSUPERVISOR_CHEMISTRY=lfp or nmc "
        "(the pack's: profiles/builtin/<profile>.h includes chemistry/<chemistry>.h)")
endif()
message(STATUS "Supervisor chemistry: ${SUPERVISOR_CHEMISTRY}")

# The board only matters for its USB wiring (USB_VBUS_SENSE,
# USB_EXTERNAL_PULLUP): use the same BMS_BOARD as the BMS
include(${BMS_ROOT}/boards/boards.cmake)
set(BMS_BOARD "" CACHE STRING "Board to build for (see boards/README.md)")

add_executable(supervisor
    isospi/isosnoop.c
    cell_snoop.c
    supervisor_logic.c
    supervisor_hmi.c
    supervisor.c
    ${BMS_DIR}/shared/cell_map.c
    ${BMS_DIR}/shared/supervisor_protocol.c
    ${BMS_DIR}/drivers/bmb3y/crc.c
    )

bms_use_board(supervisor "${BMS_BOARD}")
message(STATUS "Board: ${BMS_BOARD}")

target_include_directories(supervisor PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${BMS_DIR}
    ${BMS_ROOT}/profiles
)

# The BMS's snooper program
pico_generate_pio_header(supervisor ${BMS_DIR}/drivers/isospi/isosnoop.pio)

target_link_libraries(supervisor pico_stdlib hardware_dma hardware_pio hardware_uart)

target_compile_definitions(supervisor PRIVATE
    SUPERVISOR_CHEMISTRY_HEADER="chemistry/${SUPERVISOR_CHEMISTRY}.h"
    USBD_MANUFACTURER="S3XY-BMS"
    USBD_PRODUCT="Supervisor"
    # Drop USB output rather than wait long for a host that isn't reading (the
    # SDK default is 500ms per write). 5ms lets a status burst drain to a host
    # that is reading, while the snooper DMA and UART RX ring cover the wait.
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=5000
)

target_compile_options(supervisor PRIVATE -Wall -Wextra)

# No C++ (as for the BMS)
target_compile_definitions(supervisor PRIVATE PICO_CXX_DISABLE_ALLOCATION_OVERRIDES=1)
set_target_properties(supervisor PROPERTIES LINKER_LANGUAGE C)

pico_set_program_description(supervisor "Supervisor, chemistry: ${SUPERVISOR_CHEMISTRY}, board: ${BMS_BOARD}")

# enable usb output, disable uart output
pico_enable_stdio_usb(supervisor 1)
pico_enable_stdio_uart(supervisor 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(supervisor)
