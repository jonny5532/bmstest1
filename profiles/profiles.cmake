# Pack profile selection, to specify the defaults, limits and pack layout.
#
#   -DBMS_PROFILE=<name>      profile to build (file name without .h)
#   -DBMS_PROFILE_DIR=<dir>   optional extra directory of custom profiles,
#                             e.g. one kept outside the repo
#
# The BMS_PROFILE_DIR, profiles/custom and profiles/builtin folders are searched
# for the requested profile.

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/select_header.cmake)

set(BMS_PROFILES_ROOT ${CMAKE_CURRENT_LIST_DIR})

set(BMS_PROFILE "bluetesla" CACHE STRING "Pack profile to build (see profiles/README.md)")
set(BMS_PROFILE_DIR "" CACHE PATH "Extra directory to search for custom pack profiles")

bms_select_header("pack profile" BMS_PROFILE "${BMS_PROFILE}"
    "${BMS_PROFILE_DIR};${BMS_PROFILES_ROOT}/custom;${BMS_PROFILES_ROOT}/builtin"
    "(Profiles are named by file now: BMS_LFP1 is lfp1, and so on.)"
    BMS_PROFILE_HEADER)
message(STATUS "Pack profile: ${BMS_PROFILE} (${BMS_PROFILE_HEADER})")

add_compile_definitions(
    BMS_PROFILE_HEADER="${BMS_PROFILE_HEADER}"
    BMS_PROFILE_NAME="${BMS_PROFILE}"
)
include_directories(${BMS_PROFILES_ROOT})
