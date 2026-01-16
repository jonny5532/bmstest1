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
    ../monitoring/events.c
    ../state_machines/contactors.c 
    ../state_machines/base.c
)
target_link_libraries(test_contactors PRIVATE cmocka)
target_include_directories(test_contactors PRIVATE 
    ${cmocka_SOURCE_DIR}/include
    include
    ..
)

add_test(NAME test_contactors COMMAND ${MEMORY_CHECK} test_contactors)
