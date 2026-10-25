# Picks the tools the Raspberry Pi Pico VS Code extension installs, so a build
# needs nothing on PATH and no tool versions written down anywhere. Included
# before pico_sdk_init.cmake, and only does anything when PICO_EXTENSION_DIR is
# set (the windows preset sets it to %USERPROFILE%/.pico-sdk).
#
#   pioasm, picotool: the versions matching the Pico SDK (PICO_SDK_DIR)
#   Arm GCC, Ninja:   the newest installed
#
# Anything already set (on the command line or in the cache) is left alone.

if(NOT PICO_EXTENSION_DIR)
    return()
endif()

include(${PICO_SDK_DIR}/pico_sdk_version.cmake)
set(_sdk ${PICO_SDK_VERSION_MAJOR}.${PICO_SDK_VERSION_MINOR}.${PICO_SDK_VERSION_REVISION})

# Newest match of a glob (natural order, so 14_2 sorts after 13_3 and 2.10 after 2.9)
function(_pico_ext_newest out glob)
    file(GLOB _found LIST_DIRECTORIES true ${glob})
    if(_found)
        list(SORT _found COMPARE NATURAL ORDER DESCENDING)
        list(GET _found 0 _found)
    endif()
    set(${out} "${_found}" PARENT_SCOPE)
endfunction()

function(_pico_ext_set var path what)
    if(DEFINED ${var} OR DEFINED CACHE{${var}})
        return()
    endif()
    if(NOT path OR NOT EXISTS "${path}")
        message(FATAL_ERROR "No ${what} found in ${PICO_EXTENSION_DIR} for Pico SDK ${_sdk}: "
                            "install it with the Pico VS Code extension, or set ${var}")
    endif()
    set(${var} "${path}" CACHE PATH "${what} (from the Pico VS Code extension)")
    message(STATUS "Pico extension: ${what}: ${path}")
endfunction()

_pico_ext_newest(_ninja "${PICO_EXTENSION_DIR}/ninja/*/ninja.exe")
_pico_ext_set(CMAKE_MAKE_PROGRAM "${_ninja}" "Ninja")

# Arm toolchains only: the RISC-V one sits in the same folder
_pico_ext_newest(_gcc "${PICO_EXTENSION_DIR}/toolchain/*/bin/arm-none-eabi-gcc.exe")
if(_gcc)
    get_filename_component(_gcc "${_gcc}" DIRECTORY)
    get_filename_component(_gcc "${_gcc}" DIRECTORY)
endif()
_pico_ext_set(PICO_TOOLCHAIN_PATH "${_gcc}" "Arm GCC")

_pico_ext_set(pioasm_DIR "${PICO_EXTENSION_DIR}/tools/${_sdk}/pioasm" "pioasm")

# picotool versions carry a suffix (2.2.0-a4)
_pico_ext_newest(_picotool "${PICO_EXTENSION_DIR}/picotool/${_sdk}*/picotool")
_pico_ext_set(picotool_DIR "${_picotool}" "picotool")
