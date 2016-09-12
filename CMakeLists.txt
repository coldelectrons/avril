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

add_avr_library(avril
    ./devices/wii_nunchuk.cc
    ./i2c/i2c.cc
    ./serial.cc
    ./adc.cc
    ./time.cc
    ./string.cc
    ./filesystem/directory.cc
    ./filesystem/file.cc
    ./filesystem/filesystem.cc
    ./random.cc
    ./ff/rtc.c
    ./ff/ff.c
    ./ff/mmc.c
)
