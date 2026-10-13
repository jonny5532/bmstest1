cmake_minimum_required(VERSION 3.28)
enable_testing()

project(UnitTests LANGUAGES C)

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
    ../bms/app/state_machines/base.c
)
target_compile_definitions(test_contactors_rev1 PRIVATE BMS_BOARD=2)
target_link_libraries(test_contactors_rev1 PRIVATE cmocka)
target_include_directories(test_contactors_rev1 PRIVATE 
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_contactors_rev1 COMMAND ${MEMORY_CHECK} test_contactors_rev1)

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

add_executable(test_events 
    test_events.c 
    ../bms/sys/events/events.c
    ../bms/app/battery/current_limits.c
    ../bms/app/battery/safety_checks.c
    ../bms/app/estimators/ekf.c
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

add_executable(test_system_sm
    test_system_sm.c
    ../bms/sys/events/events.c
    ../bms/app/state_machines/system.c
    ../bms/app/state_machines/base.c
    ../bms/app/model.c
    ../bms/app/battery/current_limits.c
    physical_model.c
)
target_link_libraries(test_system_sm PRIVATE cmocka m)
target_include_directories(test_system_sm PRIVATE 
    ${cmocka_SOURCE_DIR}/include
    include
    ../bms
)

add_test(NAME test_system_sm COMMAND ${MEMORY_CHECK} test_system_sm)

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

add_custom_target(coverage
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/coverage_report.sh ${CMAKE_CURRENT_BINARY_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Run built tests and generate gcov coverage summary"
)
