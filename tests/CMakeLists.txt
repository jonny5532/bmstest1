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

add_executable(test_soc
    test_soc.c
    ../bms/app/model.c
    ../bms/app/battery/current_limits.c
    ../bms/app/estimators/ekf.c
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
