cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# --------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON) no need for this, MX already generate that code.

# --------------


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME slam_hardware)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(CXX C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME} custom_main.cpp)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ./lib/stdlib
    ./lib/algorithm
    ./lib/mpu6050_cpp
    ./lib/n10p/
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)


# -----------lib
add_library(ssd1306 STATIC
    ./lib/ssd1306-stm32HAL/lib/ssd1306.c
    ./lib/ssd1306-stm32HAL/lib/fonts.c)

target_include_directories(ssd1306 INTERFACE
    ./lib/ssd1306-stm32HAL/lib)

target_link_libraries(ssd1306 stm32cubemx)

add_library(mpu6050 STATIC ./lib/MPU6050/Src/mpu6050.c)
target_include_directories(mpu6050 INTERFACE ./lib/MPU6050/Src/)
target_link_libraries(mpu6050  stm32cubemx)
# ---------------- 


# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ssd1306
    mpu6050
    # Add user defined libraries
)
