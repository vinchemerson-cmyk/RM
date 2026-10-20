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


# Define the build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif ()

# Set the project name
set(CMAKE_PROJECT_NAME infantry_H723)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Application/Robot.cpp
        Application/RobotConfig.hpp
        Application/shoot/shoot.cpp
        Application/shoot/shoot.h
        Application/shoot/ShootConfig.hpp
        Application/shoot/shoot_control.h
        Modules/BUZZER/buzzer_task.cpp
        Modules/BUZZER/buzzer_task.h
        BSP/BUZZER/BUZZER.cpp
        BSP/BUZZER/BUZZER.h
        Application/Robot.h
        BSP/can/can.cpp
        BSP/can/can.h
        BSP/can/canConfig.h
        Application/gimbal/gimbal.cpp
        Application/gimbal/gimbal.h
        Application/gimbal/GimbalConfig.hpp
        BSP/imu/Bmi088.cpp
        BSP/imu/Bmi088.h
        Modules/imu/AttitudeEstimator.cpp
        Modules/imu/AttitudeEstimator.h
        Modules/imu/ImuTask.cpp
        Modules/imu/ImuTask.h
        Modules/imu/Bmi088Heater.cpp
        Modules/imu/Bmi088Heater.h
        Modules/algorithm/pid.cpp
        Modules/algorithm/pid.h
        Modules/algorithm/lqi_config.h
        Modules/algorithm/lqi.h
        Modules/algorithm/lqi.cpp
        Modules/algorithm/lqi_controller.h
        Modules/algorithm/lqi_controller.cpp
        Modules/communication/usb_serial_task.cpp
        Modules/communication/usb_serial_task.h
        Modules/communication/uart_serial_task.cpp
        Modules/communication/uart_serial_task.h
        Modules/communication/encoder_serial_task.cpp
        Modules/communication/encoder_serial_task.h
        Modules/communication/encoder_telemetry.h
        Modules/communication/interboard.cpp
        Modules/communication/interboard.h
        Modules/rc/rc.cpp
        Modules/rc/rc.h
        Modules/safety/safety.cpp
        Modules/safety/safety.h
        BSP/usart/usart.cpp
        Modules/message_center/message_center.c
        BSP/log/bsp_log.c
        Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT.c
        Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_printf.c
        Modules/motor/dji/dji.cpp
        Modules/motor/dji/dji.h
        Modules/motor/motor_common.cpp
        Modules/motor/motor_common.h
        Modules/motor/angle_target.h
        Application/chassis/chassis.cpp
        Application/chassis/chassis.h
        Modules/motor/dm/dm.h
        Modules/motor/dm/dmmotor.h
        Modules/motor/dm/dmmotor.cpp
)

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
        ${CMAKE_CURRENT_SOURCE_DIR}/Application
        ${CMAKE_CURRENT_SOURCE_DIR}/Modules
        ${CMAKE_CURRENT_SOURCE_DIR}/BSP
        ${CMAKE_CURRENT_SOURCE_DIR}/BSP/log
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/SEGGER/RTT
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/SEGGER/Config
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx

        # Add user defined libraries
)
