# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
<<<<<<< HEAD
set(PICO_BOARD pico2_w CACHE STRING "Board type")
=======
set(PICO_BOARD pico2 CACHE STRING "Board type")
>>>>>>> d2f0005967735bef039a6d9cd52b00b7d3cb66f0

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

<<<<<<< HEAD
project(firmware C CXX ASM)
=======
<<<<<<<< HEAD:3_trabajos_practicos/tp2/I2C_Scaner/CMakeLists.txt
project(I2C_Scaner C CXX ASM)
========
project(firmware C CXX ASM)
>>>>>>>> d2f0005967735bef039a6d9cd52b00b7d3cb66f0:3_trabajos_practicos/tp2/firmware/CMakeLists.txt
>>>>>>> d2f0005967735bef039a6d9cd52b00b7d3cb66f0

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add external FreeRTOS library
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../freertos ${CMAKE_BINARY_DIR}/freertos)

# Add executable. Default name is the project name, version 0.1

<<<<<<< HEAD
=======
<<<<<<<< HEAD:3_trabajos_practicos/tp2/I2C_Scaner/CMakeLists.txt
add_executable(I2C_Scaner I2C_Scaner.c )

pico_set_program_name(I2C_Scaner "I2C_Scaner")
pico_set_program_version(I2C_Scaner "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(I2C_Scaner 0)
pico_enable_stdio_usb(I2C_Scaner 1)

# Add the standard library to the build
target_link_libraries(I2C_Scaner

        pico_stdlib)

# Add the standard include files to the build
target_include_directories(I2C_Scaner PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(I2C_Scaner 
        hardware_i2c
        
        )

pico_add_extra_outputs(I2C_Scaner)
========
>>>>>>> d2f0005967735bef039a6d9cd52b00b7d3cb66f0
add_executable(firmware firmware.c )

pico_set_program_name(firmware "firmware")
pico_set_program_version(firmware "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(firmware 0)
pico_enable_stdio_usb(firmware 1)

# Add the standard library to the build
target_link_libraries(firmware
<<<<<<< HEAD
        pico_stdlib)
=======
    hardware_adc 
    freertos
    pico_stdlib)
>>>>>>> d2f0005967735bef039a6d9cd52b00b7d3cb66f0

# Add the standard include files to the build
target_include_directories(firmware PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

<<<<<<< HEAD
# Add any user requested libraries
target_link_libraries(firmware
    hardware_adc 
    freertos
        )

pico_add_extra_outputs(firmware)
=======
pico_add_extra_outputs(firmware)
>>>>>>>> d2f0005967735bef039a6d9cd52b00b7d3cb66f0:3_trabajos_practicos/tp2/firmware/CMakeLists.txt
>>>>>>> d2f0005967735bef039a6d9cd52b00b7d3cb66f0

