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
set(CMAKE_PROJECT_NAME JiuXian_SP_gimbal_Xpp)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        IT/Src/it.cpp
        IT/Inc/it.h
        Drv/WS2812/Src/Drv_ws2812.c
        Drv/WS2812/Inc/Drv_ws2812.h
        App/InitAndCheck/Src/Init_node.cpp
        App/InitAndCheck/Inc/Init_node.h
        PreApp/WS2812/Src/Indicator.cpp
        PreApp/WS2812/Inc/Indicator.h
        Config/JiuXian_Gimbal_Config.hpp
        Config/JiuXian_Gimbal_Config.cpp
        Bsp/FDCAN/Src/Bsp_fdcan.c
        Bsp/FDCAN/Inc/Bsp_fdcan.h
        App/FireControl/Src/FC.cpp
        App/FireControl/Inc/FC.h
        App/Interaction/Src/Interaction_node.cpp
        App/Interaction/Inc/Interaction_node.h
        App/Sound/Src/buzzer_node.cpp
        App/Sound/Inc/buzzer_node.h
        Drv/Buzzer/Src/Drv_buzzer.c
        Drv/Buzzer/Inc/Drv_buzzer.h
        Algorithm/PID/Src/PID.cpp
        Algorithm/PID/Inc/PID.hpp
        Algorithm/MathUnit/Src/MathUnit.cpp
        Algorithm/MathUnit/Inc/MathUnit.hpp
        Drv/DJI/Src/Drv_DJI.c
        Drv/DJI/Inc/Drv_DJI.h
        Drv/UART_Double_Buffer/Src/drv_uart_double_buffer.c
        Drv/UART_Double_Buffer/Inc/drv_uart_double_buffer.h
        Drv/RoboMaster_Referee_System/Standard_Link/Src/drv_robomaster_standard_link.c
        Drv/RoboMaster_Referee_System/Standard_Link/Inc/drv_robomaster_standard_link.h
        Drv/RoboMaster_Referee_System/VTM_Link/Src/drv_robomaster_vtm_link.c
        Drv/RoboMaster_Referee_System/VTM_Link/Inc/drv_robomaster_vtm_link.h
        App/RoboMaster_Referee_node/Standard_Link_node/Src/RoboMaster_standard_link_node.cpp
        App/RoboMaster_Referee_node/Standard_Link_node/Inc/RoboMaster_standard_link_node.h
        App/RoboMaster_Referee_node/VTM_Link_node/Src/RoboMaster_vtm_link_node.cpp
        App/RoboMaster_Referee_node/VTM_Link_node/Inc/RoboMaster_vtm_link_node.h
        Drv/RoboMaster_Referee_System/Referee_Info_Define/Inc/RoboMaster_Referee_Info_Define.h
        Drv/RoboMaster_Referee_System/Referee_Info_Define/Src/RoboMaster_CRC.c
        Drv/RoboMaster_Referee_System/Referee_Info_Define/Inc/RoboMaster_CRC.h
        PreApp/Vofa/Src/app_vofa.cpp
        PreApp/Vofa/Inc/app_vofa.hpp
        PreApp/Vofa/Inc/vofa_ascii.h
        Drv/MiniPC/Src/Drv_MiniPC.c
        Drv/MiniPC/Inc/Drv_MiniPC.h
        Drv/MiniPC/Inc/MiniPC_CRC.h
        Drv/MiniPC/Src/MiniPC_CRC.c
        App/MiniPC/Src/MiniPC_node.cpp
        App/MiniPC/Inc/MiniPC_node.h
        App/RemoteControl/Src/RemoteCtrl_node.cpp
        App/RemoteControl/Inc/RemoteCtrl_node.h
        Drv/WFLY/Src/Drv_WFLY.c
        Drv/WFLY/Inc/Drv_WFLY.h
        App/GimbalControl/Src/GC.cpp
        App/GimbalControl/Inc/GC.h
        Algorithm/SMC/Src/alg_smc.cpp
        Algorithm/SMC/Inc/alg_smc.hpp
        Config/JiuXuan_Status.hpp
        PreApp/Signal_Generator/Src/Signal_Generator.cpp
        PreApp/Signal_Generator/Inc/Signal_Generator.hpp
        App/HIPNUC/Src/imu_node.cpp
        App/HIPNUC/Inc/imu_node.h
        Drv/HIPNUC/Hl14S3/Src/drv_hipnuc_hl14s3.c
        Drv/HIPNUC/Hl14S3/Inc/drv_hipnuc_hl14s3.h
        Algorithm/S_Curve/Inc/alg_s_curve.hpp
        Algorithm/LADRC/Inc/LADRC.hpp
        Algorithm/ComplianceControl/Inc/ComplianceControl.hpp
        Algorithm/S_Curve/Inc/alg_S_Curve_Planner.hpp
        Algorithm/Ramp_Planner/Ramp.hpp
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
