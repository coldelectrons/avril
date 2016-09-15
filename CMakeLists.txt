# Since Avril is not header-only, build a lib object from source files
# This is intended to work with https://github.com/mkleemann/cmake-avr
# and be used as a git submodule in another project

add_definitions(-I.)
add_definitions(-g)
add_definitions(-Os)
add_definitions(-w)
add_definitions(-Wall)
add_definitions(-D__PROG_TYPES_COMPAT__)
add_definitions(-fdata-sections)
add_definitions(-ffunction-sections)
add_definitions(-fshort-enums)
add_definitions(-fno-move-loop-invariants)
add_definitions(${EXTRA_DEFINES})
add_definitions(${MMC_CONFIG})
add_definitions(-DSERIAL_RX_0)
add_definitions(-mcall-prologues)
add_definitions(-fno-exceptions)

set()

add_avr_library(avril
    serial.cc
    adc.cc
    time.cc
    string.cc
    random.cc
    ${CMAKE_CURRENT_LIST_DIR}/i2c/i2c.cc
    ${CMAKE_CURRENT_LIST_DIR}/devices/wii_nunchuk.cc
    ${CMAKE_CURRENT_LIST_DIR}/filesystem/directory.cc
    ${CMAKE_CURRENT_LIST_DIR}/filesystem/file.cc
    ${CMAKE_CURRENT_LIST_DIR}/filesystem/filesystem.cc
    ${CMAKE_CURRENT_LIST_DIR}/ff/rtc.c
    ${CMAKE_CURRENT_LIST_DIR}/ff/ff.c
    ${CMAKE_CURRENT_LIST_DIR}/ff/mmc.c
)
