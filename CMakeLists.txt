SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.20)

# set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_C_STANDARD 11)
# set(CMAKE_CXX_STANDARD 20)

# This two lines to avoid error 
# arm-none-eabi-g++: error: unrecognized command line option '-rdynamic'
# during compiler check. For some reasons test use '-rdynamic' by default, but arm-none-eabi-g* compilers does not support it
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

cmake_path(GET CMAKE_CURRENT_SOURCE_DIR STEM PROJECT_NAME)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(ARCHITECTURE "rv32imac")
set(INTEGER_ABI "ilp32")
set(CODE_MODEL "medany")
set(SMALL_DATA_LIMIT "8")


SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/LD/Link.ld)


SET(CMAKE_C_FLAGS_INIT "-march=${ARCHITECTURE} -mabi=${INTEGER_ABI} -mcmodel=${CODE_MODEL} -msmall-data-limit=${SMALL_DATA_LIMIT} -mno-save-restore -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections")

PROJECT(${PROJECT_NAME} C ASM)


add_definitions(
    # -DSTM32G0B1xx
)

if ((CMAKE_BUILD_TYPE MATCHES "Debug") OR (CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo"))
    add_definitions(
        -DDEBUG=1
    )
endif()

file(GLOB_RECURSE SOURCES 
    "src/*.c"
    "Startup/*.S"
    "RVMSIS/*.c"
    "StdPeriphDriver/*.c"
)

include_directories(
    "src"
    "RVMSIS"
    "StdPeriphDriver/inc"

)


# -nostartfiles -Xlinker --gc-sections -L"../" -L"../StdPeriphDriver" -Xlinker --print-memory-usage -Wl,-Map,"CH582M.map" --specs=nano.specs --specs=nosys.specs -o "CH582M.elf"  ./src/Main.o  ./StdPeriphDriver/CH58x_clk.o ./StdPeriphDriver/CH58x_flash.o ./StdPeriphDriver/CH58x_gpio.o ./StdPeriphDriver/CH58x_i2c.o ./StdPeriphDriver/CH58x_pwr.o ./StdPeriphDriver/CH58x_spi1.o ./StdPeriphDriver/CH58x_sys.o ./StdPeriphDriver/CH58x_uart0.o ./StdPeriphDriver/CH58x_uart1.o ./StdPeriphDriver/CH58x_usb2dev.o ./StdPeriphDriver/CH58x_usb2hostBase.o ./StdPeriphDriver/CH58x_usb2hostClass.o  ./Startup/startup_CH583.o  ./RVMSIS/core_riscv.o   -lISP583
# riscv-none-embed-objcopy -O ihex "CH582M.elf"  "CH582M.hex"
add_link_options(-T ${LINKER_SCRIPT})
add_link_options(-nostartfiles -Xlinker --gc-sections)
add_link_options(-Wl,--print-memory-usage,-Map,${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(--specs=nano.specs --specs=nosys.specs)



add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

find_library(ISP583_LIB ISP583 "StdPeriphDriver")

target_link_libraries(${PROJECT_NAME}.elf ${ISP583_LIB})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
