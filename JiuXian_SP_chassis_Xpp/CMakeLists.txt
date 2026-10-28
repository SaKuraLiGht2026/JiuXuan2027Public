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
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME JiuXian_SP_chassis_Xpp)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Bsp/FDCAN/Src/bsp_fdcan.c
        Bsp/FDCAN/Inc/bsp_fdcan.h
        Node/Ctrl_node/Src/app_ctrl.cpp
        Node/Ctrl_node/Inc/app_ctrl.h
        Node/IT_node/Src/app_it.cpp
        Node/IT_node/Inc/app_it.h
        Drv/DJI/Src/drv_DJI.c
        Drv/DJI/Inc/drv_DJI.h
        Algorithm/PID/Src/alg_pid.cpp
        Algorithm/PID/Inc/alg_pid.hpp
        Drv/HIPNUC/Src/drv_HIPNUC_Hl12S2.cpp
        Drv/HIPNUC/Inc/drv_HIPNUC_Hl12S2.h
        Config/Src/JiuXian_Config.cpp
        Config/Inc/JiuXian_Config.hpp
        Node/IMU_node/Src/app_imu.cpp
        Node/IMU_node/Inc/app_imu.h
        Drv/UART_Double_Buffer/Src/drv_uart_double_buffer.c
        Drv/UART_Double_Buffer/Inc/drv_uart_double_buffer.h
        Applications/Vofa/Src/app_vofa.cpp
        Applications/Vofa/Inc/app_vofa.hpp
        Node/Init_node/Src/InitAndState_node.cpp
        Node/Init_node/Inc/InitAndState_node.h
        Applications/Vofa/Inc/vofa_ascii.h
        Node/Debug_node/Src/app_debug.cpp
        Node/Debug_node/Inc/app_debug.h
        Applications/WS2812/Src/Indicator.cpp
        Drv/WS2812/Src/Drv_ws2812.c
        Node/Interaction_node/Src/app_interaction_node.cpp
        Node/Interaction_node/Inc/app_interaction_node.h
        Drv/RoboMaster_Referee_System/VTM_Link/Src/drv_robomaster_vtm_link.c
        Drv/RoboMaster_Referee_System/VTM_Link/Inc/drv_robomaster_vtm_link.h
        Drv/RoboMaster_Referee_System/Standard_Link/Src/drv_robomaster_standard_link.c
        Drv/RoboMaster_Referee_System/Standard_Link/Inc/drv_robomaster_standard_link.h
        Algorithm/SMC/Src/alg_smc.cpp
        Algorithm/SMC/Inc/alg_smc.hpp
        Algorithm/MathUnit/Src/MathUnit.cpp
        Algorithm/MathUnit/Inc/MathUnit.hpp
        Algorithm/S_Curve/Inc/alg_s_curve.hpp
        Algorithm/Ramp_Planner/Ramp.hpp
        Bsp/RS485/Src/bsp_rs485.c
        Bsp/RS485/Inc/bsp_rs485.h
        Drv/XiaoHe/Src/drv_XiaoHe.cpp
        Drv/XiaoHe/Inc/drv_XiaoHe.h
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
#
## 使能printf浮点数支持
#target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
