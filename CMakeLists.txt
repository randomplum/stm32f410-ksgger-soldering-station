cmake_minimum_required(VERSION 3.7.0)

set(CMAKE_VERBOSE_MAKEFILE on)
set(CMAKE_COLOR_MAKEFILE on)
set(CMAKE_ERROR_DEPRECATED on)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
SET(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
enable_language(C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_OSX_SYSROOT ${CMAKE_CURRENT_SOURCE_DIR})

project(stm32f4)
set(TARGET ${PROJECT_NAME}.elf)

set(PROCESSOR_FAMILY ARM_CM4)
set(STM32_FAMILY "F4")
set(STM32CUBEDIR "STM32CubeF4")
string(TOLOWER ${STM32_FAMILY} STM32_FAMILY_LC)
set(MCU_LD_NAME "STM32F410RBTx")
set(MCU_NAME "STM32F410Rx")
string(TOLOWER ${MCU_NAME} MCU_NAME_LC)

set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

#IF(NOT CROSS_COMPILE_PREFIX)
#    SET(CROSS_COMPILE_PREFIX "arm-none-eabi-")
#    MESSAGE(WARNING "No CROSS_COMPILE_PREFIX specified, using default: " ${CROSS_COMPILE_PREFIX})
#ENDIF()

#SET(CMAKE_C_COMPILER "${CROSS_COMPILE_PREFIX}gcc")
#set(CMAKE_ASM_COMPILER "${CROSS_COMPILE_PREFIX}gcc")
#SET(CMAKE_OBJCOPY "${CROSS_COMPILE_PREFIX}objcopy" CACHE INTERNAL "objcopy tool")
#SET(CMAKE_OBJDUMP "${CROSS_COMPILE_PREFIX}objdump" CACHE INTERNAL "objdump tool")
SET(CMAKE_SIZE "arm-none-eabi-size${CMAKE_EXECUTABLE_SUFFIX}" CACHE INTERNAL "size tool")

# Set a default build type if none was specified
IF( NOT CMAKE_BUILD_TYPE )
  SET(default_build_type "Release")
  MESSAGE(WARNING "Setting build type to '${default_build_type}' as none was specified.")
  SET(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
ENDIF()

SET(CMAKE_C_FLAGS " -march=armv7e-m -mthumb -fno-builtin -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Wall -DNDEBUG -D${MCU_NAME} -std=gnu99 -ffunction-sections -fdata-sections -fomit-frame-pointer -mabi=aapcs -ffast-math -ftree-vectorize" CACHE INTERNAL "c compiler flags")
SET(CMAKE_ASM_FLAGS " -march=armv7e-m -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Wa,--no-warn -Wall -DNDEBUG -D${MCU_NAME} -x assembler-with-cpp" CACHE INTERNAL "asm compiler flags")

SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -mthumb --specs=nano.specs --specs=nosys.specs -Wl,-Map,\"${TARGET}.map\"" CACHE INTERNAL "executable linker flags")

SET(CMAKE_C_FLAGS_DEBUG "-Og -g" CACHE INTERNAL "c compiler flags debug")
SET(CMAKE_ASM_FLAGS_DEBUG "-g" CACHE INTERNAL "asm compiler flags debug")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "" CACHE INTERNAL "linker flags debug")

SET(CMAKE_C_FLAGS_RELEASE "-Os -flto" CACHE INTERNAL "c compiler flags release")
SET(CMAKE_ASM_FLAGS_RELEASE "" CACHE INTERNAL "asm compiler flags release")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "-flto" CACHE INTERNAL "linker flags release")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/${STM32CUBEDIR}/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/${STM32CUBEDIR}/Drivers/STM32${STM32_FAMILY}xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/${STM32CUBEDIR}/Drivers/CMSIS/Device/ST/STM32${STM32_FAMILY}xx/Include/
    ${CMAKE_CURRENT_SOURCE_DIR}/app/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/board/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/common/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/generalIO
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/graphics
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/graphics/gui
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/graphics/gui/screens
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/graphics/u8g2
    )

set(SOURCES
   ${STM32CUBEDIR}/Drivers/CMSIS/Device/ST/STM32${STM32_FAMILY}xx/Source/Templates/gcc/startup_${MCU_NAME_LC}.s
   ${STM32CUBEDIR}/Drivers/CMSIS/Device/ST/STM32${STM32_FAMILY}xx/Source/Templates/system_stm32${STM32_FAMILY_LC}xx.c
   common/src/syscalls.c
   common/src/stm32${STM32_FAMILY_LC}xx_it.c
   )


set(STM32CUBE_HAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${STM32CUBEDIR}/Drivers/STM32${STM32_FAMILY}xx_HAL_Driver/)
list(APPEND SOURCES
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal.c
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal_cortex.c
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal_gpio.c
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal_pwr.c
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal_rcc.c
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal_rcc_ex.c
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal_tim.c
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal_tim_ex.c
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal_uart.c
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal_usart.c
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal_adc.c
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal_crc.c
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal_iwdg.c
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal_dma.c
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal_flash.c
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal_adc_ex.c
    ${STM32CUBE_HAL_DIR}/Src/stm32${STM32_FAMILY_LC}xx_hal_flash_ex.c
)
include(app/CMakeLists.txt)
include(board/CMakeLists.txt)
include(drivers/CMakeLists.txt)

add_executable(${TARGET} ${SOURCES})

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/board/${MCU_LD_NAME}_FLASH.ld ${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_flash.ld)

GET_TARGET_PROPERTY(TARGET_LD_FLAGS ${TARGET} LINK_FLAGS)
IF(TARGET_LD_FLAGS)
    SET(TARGET_LD_FLAGS "-Wl,--print-memory-usage \"-T${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_flash.ld\" ${TARGET_LD_FLAGS}")
ELSE()
    SET(TARGET_LD_FLAGS "-Wl,--print-memory-usage \"-T${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_flash.ld\"")
ENDIF()
SET_TARGET_PROPERTIES(${TARGET} PROPERTIES LINK_FLAGS ${TARGET_LD_FLAGS})

SET(FILENAME "${TARGET}")
# ADD_CUSTOM_TARGET(${TARGET}.bin ALL DEPENDS ${TARGET} COMMAND ${CMAKE_OBJCOPY} -Obinary ${FILENAME} ${FILENAME}.bin)
add_custom_command(TARGET ${TARGET} POST_BUILD COMMAND ${CMAKE_SIZE} ${FILENAME})
