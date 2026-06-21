set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Arm GNU Toolchain must be available on PATH. An explicit installation can
# also be selected by configuring with -DARM_TOOLCHAIN_ROOT=<toolchain root>.
set(ARM_TOOLCHAIN_ROOT "" CACHE PATH "Arm GNU Toolchain installation root")
if(ARM_TOOLCHAIN_ROOT)
    set(TOOLCHAIN_BIN_HINT "${ARM_TOOLCHAIN_ROOT}/bin")
endif()

find_program(CMAKE_C_COMPILER
    NAMES arm-none-eabi-gcc
    HINTS "${TOOLCHAIN_BIN_HINT}"
    REQUIRED)
find_program(CMAKE_ASM_COMPILER
    NAMES arm-none-eabi-gcc
    HINTS "${TOOLCHAIN_BIN_HINT}"
    REQUIRED)
find_program(CMAKE_CXX_COMPILER
    NAMES arm-none-eabi-g++
    HINTS "${TOOLCHAIN_BIN_HINT}"
    REQUIRED)
find_program(CMAKE_LINKER
    NAMES arm-none-eabi-g++
    HINTS "${TOOLCHAIN_BIN_HINT}"
    REQUIRED)
find_program(CMAKE_OBJCOPY
    NAMES arm-none-eabi-objcopy
    HINTS "${TOOLCHAIN_BIN_HINT}"
    REQUIRED)
find_program(CMAKE_SIZE
    NAMES arm-none-eabi-size
    HINTS "${TOOLCHAIN_BIN_HINT}"
    REQUIRED)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
