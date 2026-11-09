# Board selection: the PCB revision, plus any modifications made to a
# particular board (see boards/README.md).
#
#   -DBMS_BOARD=<name>        board to build for (file name without .h)
#   -DBMS_BOARD_DIR=<dir>     optional extra directory of custom boards
#
# The BMS_BOARD_DIR, boards/custom and boards/builtin folders are searched for
# the requested board.

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/select_header.cmake)

set(BMS_BOARDS_ROOT ${CMAKE_CURRENT_LIST_DIR})

set(BMS_BOARD_DIR "" CACHE PATH "Extra directory to search for custom boards")

# Build target for the named board
function(bms_use_board target name)
    if(name STREQUAL "")
        bms_list_headers("${BMS_BOARD_DIR};${BMS_BOARDS_ROOT}/custom;${BMS_BOARDS_ROOT}/builtin" available)
        message(FATAL_ERROR "Set the board with -DBMS_BOARD=<name>. Available: ${available}\n"
            "Use a modified board's own name (e.g. desk): its USB wiring can't be detected, "
            "and the wrong setting stops USB connecting.")
    endif()
    bms_select_header(board BMS_BOARD "${name}"
        "${BMS_BOARD_DIR};${BMS_BOARDS_ROOT}/custom;${BMS_BOARDS_ROOT}/builtin"
        "(Boards are named by file now: BMS_BOARD=1 is rev0 and 2 is rev1, unless the board is modified.)"
        header)
    target_compile_definitions(${target} PRIVATE
        BMS_BOARD_HEADER="${header}"
        BMS_BOARD_NAME="${name}"
    )
    set_target_properties(${target} PROPERTIES BMS_BOARD ${name})
endfunction()

# Boards include each other as "boards/builtin/<name>.h": plain "builtin/" is
# the pack profiles'
include_directories(${BMS_BOARDS_ROOT}/..)
