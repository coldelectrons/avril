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
add_definitions(-DHAS_USART0 )
add_definitions(-mcall-prologues)
add_definitions(-fno-exceptions)

#find_library(AVRLIBC_LIBRARY
    #libc
    #PATHS
        #/usr/lib/avr/lib/
#)

#find_path(AVRLIBC_INCLUDE_DIR
    #stdlib.h
    #PATHS
        #/usr/lib/avr/include
#)

#include_directories(SYSTEM ${AVRLIBC_INCLUDE_DIR})

add_avr_library(avril
    io/adc.cc
    io/i2c.cc
    system/time.cc
    utils/stringutil.cc
    utils/random.cc
    #devices/wii_nunchuk.cc
    #filesystem/directory.cc
    #filesystem/file.cc
    #filesystem/filesystem.cc
    #third_party/ff/rtc.c
    #third_party/ff/ff.c
    #third_party/ff/mmc.c
)

target_link_libraries(avril-${AVR_MCU} ${AVRLIBC_LIBRARY} )
