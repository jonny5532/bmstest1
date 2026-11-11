cmake_minimum_required(VERSION 3.28)
enable_testing()

project(UnitTests LANGUAGES C)

include(../profiles/profiles.cmake)
include(../boards/boards.cmake)

include(FetchContent)

FetchContent_Declare(
    cmocka
    URL https://cmocka.org/files/2.0/cmocka-2.0.2.tar.xz
)

# Build CMocka statically
set(WITH_STATIC_LIB ON CACHE BOOL "Build with static library" FORCE)
set(WITH_SHARED_LIB OFF CACHE BOOL "Build with shared library" FORCE)
set(WITH_EXAMPLES OFF CACHE BOOL "Build examples" FORCE)
set(UNIT_TESTING OFF CACHE BOOL "Build internal unit tests" FORCE)

FetchContent_MakeAvailable(cmocka)

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -W -g -fms-extensions -fprofile-arcs -ftest-coverage -O0")

add_executable(test_contactors 
    test_contactors.c 
    ../bms/sys/events/events.c
    ../bms/app/state_machines/contactors.c 
    ../bms/shared/cell_map.c
    ../bms/app/state_machines/base.c
)
target_link_libraries(test_contactors PRIVATE cmocka)
target_include_directories(test_contactors PRIVATE 
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_contactors COMMAND ${MEMORY_CHECK} test_contactors)

# Rev1 board variant (dual ADS1115, precharge on negative FC contactor)
add_executable(test_contactors_rev1
    test_contactors.c 
    ../bms/sys/events/events.c
    ../bms/app/state_machines/contactors.c 
    ../bms/shared/cell_map.c
    ../bms/app/state_machines/base.c
)
bms_use_board(test_contactors_rev1 rev1)
target_link_libraries(test_contactors_rev1 PRIVATE cmocka)
target_include_directories(test_contactors_rev1 PRIVATE 
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_contactors_rev1 COMMAND ${MEMORY_CHECK} test_contactors_rev1)

# Rev1 with the precharge element on the positive side
add_executable(test_contactors_rev1_pos_pre
    test_contactors.c
    ../bms/sys/events/events.c
    ../bms/app/state_machines/contactors.c
    ../bms/shared/cell_map.c
    ../bms/app/state_machines/base.c
)
bms_use_board(test_contactors_rev1_pos_pre rev1)
target_compile_definitions(test_contactors_rev1_pos_pre PRIVATE PRECHARGE_ON_NEGATIVE=0)
target_link_libraries(test_contactors_rev1_pos_pre PRIVATE cmocka)
target_include_directories(test_contactors_rev1_pos_pre PRIVATE
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_contactors_rev1_pos_pre COMMAND ${MEMORY_CHECK} test_contactors_rev1_pos_pre)

add_executable(test_low_voltage
    test_low_voltage.c
    physical_model.c
    ../bms/sys/events/events.c
    ../bms/app/battery/safety_checks.c
    ../bms/app/battery/current_limits.c
    ../bms/app/model.c
)
target_link_libraries(test_low_voltage PRIVATE cmocka)
target_include_directories(test_low_voltage PRIVATE 
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_low_voltage COMMAND ${MEMORY_CHECK} test_low_voltage)

add_executable(dump_current_limits
    dump_current_limits.c
    ../bms/app/battery/current_limits.c
)
target_link_libraries(dump_current_limits PRIVATE cmocka)
target_include_directories(dump_current_limits PRIVATE
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_executable(test_soc
    test_soc.c
    ../bms/app/model.c
    ../bms/app/battery/current_limits.c
    ../bms/app/estimators/ekf.c
    ../bms/app/estimators/ocv_learn.c
    ../bms/app/estimators/voltage_based.c
    ../bms/protocols/inverter/can.c
    ../bms/protocols/inverter/byd_can.c
    ../bms/sys/events/events.c
    ../bms/app/state_machines/base.c
)
target_link_libraries(test_soc PRIVATE cmocka m)
target_include_directories(test_soc PRIVATE 
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
    ../vendor/can2040/src
)

add_test(NAME test_soc COMMAND ${MEMORY_CHECK} test_soc)

add_executable(test_basic_count
    test_basic_count.c
    ../bms/app/estimators/basic_count.c
)
target_link_libraries(test_basic_count PRIVATE cmocka m)
target_include_directories(test_basic_count PRIVATE
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_basic_count COMMAND ${MEMORY_CHECK} test_basic_count)

add_executable(test_lifetime
    test_lifetime.c
    ../bms/app/estimators/lifetime.c
)
target_link_libraries(test_lifetime PRIVATE cmocka m)
target_include_directories(test_lifetime PRIVATE
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_lifetime COMMAND ${MEMORY_CHECK} test_lifetime)

add_executable(test_byd_can
    test_byd_can.c
    ../bms/protocols/inverter/can.c
    ../bms/protocols/inverter/byd_can.c
    ../bms/sys/events/events.c
)
target_link_libraries(test_byd_can PRIVATE cmocka)
target_include_directories(test_byd_can PRIVATE
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
    ../vendor/can2040/src
)

add_test(NAME test_byd_can COMMAND ${MEMORY_CHECK} test_byd_can)

add_executable(test_events 
    test_events.c 
    ../bms/sys/events/events.c
    ../bms/app/battery/current_limits.c
    ../bms/app/battery/safety_checks.c
    ../bms/app/estimators/ekf.c
    ../bms/app/estimators/ocv_learn.c
    ../bms/app/hardware_checks.c
    ../bms/app/model.c
)
target_link_libraries(test_events PRIVATE cmocka m)
target_include_directories(test_events PRIVATE 
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_events COMMAND ${MEMORY_CHECK} test_events)

add_executable(test_balancing
    test_balancing.c
    ../bms/sys/events/events.c
    ../bms/app/battery/balancing.c
    ../bms/app/state_machines/base.c
    ../bms/app/model.c
    ../bms/app/battery/current_limits.c
    physical_model.c
)
target_link_libraries(test_balancing PRIVATE cmocka m)
target_include_directories(test_balancing PRIVATE 
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_balancing COMMAND ${MEMORY_CHECK} test_balancing)

add_executable(test_current_limits
    test_current_limits.c
    ../bms/app/battery/current_limits.c
)
target_link_libraries(test_current_limits PRIVATE cmocka m)
target_include_directories(test_current_limits PRIVATE 
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_current_limits COMMAND ${MEMORY_CHECK} test_current_limits)

foreach(variant IN ITEMS "" "_rev1")
    add_executable(test_system_sm${variant}
        test_system_sm.c
        ../bms/sys/events/events.c
        ../bms/app/state_machines/system.c
        ../bms/app/state_machines/base.c
        ../bms/app/model.c
        ../bms/app/battery/current_limits.c
        physical_model.c
    )
    target_link_libraries(test_system_sm${variant} PRIVATE cmocka m)
    target_include_directories(test_system_sm${variant} PRIVATE
        ${cmocka_SOURCE_DIR}/include
        include
        ../bms
    )
    add_test(NAME test_system_sm${variant} COMMAND ${MEMORY_CHECK} test_system_sm${variant})
endforeach()
bms_use_board(test_system_sm_rev1 rev1)

# Contactor offset calibration
foreach(variant IN ITEMS "" "_rev1")
    add_executable(test_calibration${variant}
        test_calibration.c
        ../bms/app/calibration/common.c
    )
    target_link_libraries(test_calibration${variant} PRIVATE cmocka m)
    target_include_directories(test_calibration${variant} PRIVATE
        ${cmocka_SOURCE_DIR}/include
        include
        ../bms
    )
    add_test(NAME test_calibration${variant} COMMAND ${MEMORY_CHECK} test_calibration${variant})
endforeach()
bms_use_board(test_calibration_rev1 rev1)

# Calibration sequences, against fake ADC and current sensor drivers
foreach(variant IN ITEMS "" "_rev1")
    add_executable(test_calibration_sm${variant}
        test_calibration_sm.c
        ../bms/app/calibration/calibration.c
        ../bms/app/calibration/common.c
        ../bms/app/state_machines/base.c
    )
    target_link_libraries(test_calibration_sm${variant} PRIVATE cmocka m)
    target_include_directories(test_calibration_sm${variant} PRIVATE
        ${cmocka_SOURCE_DIR}/include
        include
        ../bms
    )
    add_test(NAME test_calibration_sm${variant} COMMAND ${MEMORY_CHECK} test_calibration_sm${variant})
endforeach()
bms_use_board(test_calibration_sm_rev1 rev1)

# ADS1115 scan driver, against fake I2C and timer drivers
foreach(variant IN ITEMS "" "_rev1")
    add_executable(test_ads1115${variant}
        test_ads1115.c
        ../bms/drivers/sensors/ads1115.c
        ../bms/lib/sampler.c
    )
    target_link_libraries(test_ads1115${variant} PRIVATE cmocka)
    target_include_directories(test_ads1115${variant} PRIVATE
        ${cmocka_SOURCE_DIR}/include
        include
        ../bms
    )
    add_test(NAME test_ads1115${variant} COMMAND ${MEMORY_CHECK} test_ads1115${variant})
endforeach()
bms_use_board(test_ads1115_rev1 rev1)

foreach(variant IN ITEMS "" "_rev1")
    add_executable(test_board_id${variant}
        test_board_id.c
        ../bms/app/board_id.c
    )
    target_link_libraries(test_board_id${variant} PRIVATE cmocka)
    target_include_directories(test_board_id${variant} PRIVATE
        ${cmocka_SOURCE_DIR}/include
        include
        ../bms
    )
    add_test(NAME test_board_id${variant} COMMAND ${MEMORY_CHECK} test_board_id${variant})
endforeach()
bms_use_board(test_board_id_rev1 rev1)

foreach(variant IN ITEMS "" "_rev1")
    add_executable(test_isolation${variant}
        test_isolation.c
        ../bms/app/isolation.c
        ../bms/sys/events/events.c
        ../bms/app/state_machines/base.c
    )
    target_link_libraries(test_isolation${variant} PRIVATE cmocka m)
    target_include_directories(test_isolation${variant} PRIVATE
        ${cmocka_SOURCE_DIR}/include
        include
        ../bms
    )
    add_test(NAME test_isolation${variant} COMMAND ${MEMORY_CHECK} test_isolation${variant})
endforeach()
bms_use_board(test_isolation_rev1 rev1)

add_executable(test_sampler
    test_sampler.c
    ../bms/lib/sampler.c
)
target_link_libraries(test_sampler PRIVATE cmocka)
target_include_directories(test_sampler PRIVATE 
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_sampler COMMAND ${MEMORY_CHECK} test_sampler)

add_executable(test_ina228
    test_ina228.c
    ../bms/drivers/sensors/ina228.c
)
target_link_libraries(test_ina228 PRIVATE cmocka m)
target_include_directories(test_ina228 PRIVATE
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_ina228 COMMAND ${MEMORY_CHECK} test_ina228)

add_executable(test_model
    test_model.c
    ../bms/sys/events/events.c
    ../bms/app/battery/safety_checks.c
    ../bms/app/battery/current_limits.c
    ../bms/app/model.c
    physical_model.c
)
target_link_libraries(test_model PRIVATE cmocka m)
target_include_directories(test_model PRIVATE 
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_model COMMAND ${MEMORY_CHECK} test_model)

add_executable(test_state_machine_base
    test_state_machine_base.c
    ../bms/app/state_machines/base.c
)
target_link_libraries(test_state_machine_base PRIVATE cmocka)
target_include_directories(test_state_machine_base PRIVATE 
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_state_machine_base COMMAND ${MEMORY_CHECK} test_state_machine_base)

add_executable(test_events_detailed
    test_events_detailed.c
    ../bms/sys/events/events.c
)
target_link_libraries(test_events_detailed PRIVATE cmocka)
target_include_directories(test_events_detailed PRIVATE 
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_events_detailed COMMAND ${MEMORY_CHECK} test_events_detailed)

add_executable(test_hmi_arrays
    test_hmi_arrays.c
    ../bms/protocols/hmi_serial/hmi_arrays.c
)
target_link_libraries(test_hmi_arrays PRIVATE cmocka m)
target_include_directories(test_hmi_arrays PRIVATE
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_hmi_arrays COMMAND ${MEMORY_CHECK} test_hmi_arrays)

add_executable(test_hmi_registers
    test_hmi_registers.c
    ../bms/protocols/hmi_serial/hmi_registers.c
)
target_link_libraries(test_hmi_registers PRIVATE cmocka m)
target_include_directories(test_hmi_registers PRIVATE
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
    ../vendor/can2040/src
)

add_test(NAME test_hmi_registers COMMAND ${MEMORY_CHECK} test_hmi_registers)

add_executable(test_bmb3y_cells
    test_bmb3y_cells.c
)
target_link_libraries(test_bmb3y_cells PRIVATE cmocka)
target_include_directories(test_bmb3y_cells PRIVATE
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_bmb3y_cells COMMAND ${MEMORY_CHECK} test_bmb3y_cells)

add_executable(test_bmb3y_power
    test_bmb3y_power.c
    ../bms/sys/events/events.c
    ../bms/app/state_machines/base.c
)
target_link_libraries(test_bmb3y_power PRIVATE cmocka)
target_include_directories(test_bmb3y_power PRIVATE
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_bmb3y_power COMMAND ${MEMORY_CHECK} test_bmb3y_power)

add_custom_target(coverage
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/coverage_report.sh ${CMAKE_CURRENT_BINARY_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Run built tests and generate gcov coverage summary"
)

# Every other test builds for an unmodified Rev0 board
get_property(test_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
foreach(t IN LISTS test_targets)
    get_target_property(board ${t} BMS_BOARD)
    get_target_property(type ${t} TYPE)
    if(NOT board AND type STREQUAL "EXECUTABLE")
        bms_use_board(${t} rev0)
    endif()
endforeach()

# Supervisor firmware logic, its link to the BMS and the cell map. Built per
# chemistry, as the supervisor is: the cell map comes from the BMS at runtime.
foreach(chemistry IN ITEMS lfp nmc)
    add_executable(test_supervisor_${chemistry}
        test_supervisor.c
        ../bms/shared/cell_map.c
        ../bms/shared/supervisor_protocol.c
        ../bms/shared/duart_frame.c
        ../bms/drivers/bmb3y/crc.c
        ../supervisor/cell_snoop.c
        ../supervisor/supervisor_logic.c
        ../supervisor/supervisor_hmi.c
    )
    target_compile_definitions(test_supervisor_${chemistry} PRIVATE
        SUPERVISOR_CHEMISTRY_HEADER="chemistry/${chemistry}.h")
    target_link_libraries(test_supervisor_${chemistry} PRIVATE cmocka)
    target_include_directories(test_supervisor_${chemistry} PRIVATE
        ${cmocka_SOURCE_DIR}/include
        include
        ../bms
        ../supervisor
    )
    add_test(NAME test_supervisor_${chemistry} COMMAND ${MEMORY_CHECK} test_supervisor_${chemistry})
endforeach()

# Every built-in pack profile's cell map passes the supervisor's checks
file(GLOB BUILTIN_PROFILES RELATIVE ${CMAKE_CURRENT_LIST_DIR}/../profiles/builtin
    ${CMAKE_CURRENT_LIST_DIR}/../profiles/builtin/*.h)
foreach(profile_file IN LISTS BUILTIN_PROFILES)
    get_filename_component(profile ${profile_file} NAME_WE)
    add_executable(test_profile_map_${profile}
        test_profile_map.c
        ../bms/shared/cell_map.c
    )
    target_compile_definitions(test_profile_map_${profile} PRIVATE
        TEST_PROFILE_HEADER="builtin/${profile_file}" TEST_PROFILE_NAME="${profile}")
    target_link_libraries(test_profile_map_${profile} PRIVATE cmocka)
    target_include_directories(test_profile_map_${profile} PRIVATE
        ${cmocka_SOURCE_DIR}/include
        ../bms
    )
    add_test(NAME test_profile_map_${profile} COMMAND ${MEMORY_CHECK} test_profile_map_${profile})
endforeach()

# DUART framing (shared by the BMS and the supervisor)
add_executable(test_duart_frame
    test_duart_frame.c
    ../bms/shared/duart_frame.c
)
target_link_libraries(test_duart_frame PRIVATE cmocka)
target_include_directories(test_duart_frame PRIVATE
    ${cmocka_SOURCE_DIR}/include
    ../bms
)
add_test(NAME test_duart_frame COMMAND ${MEMORY_CHECK} test_duart_frame)
