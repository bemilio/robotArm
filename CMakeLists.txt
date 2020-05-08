cmake_minimum_required(VERSION 3.0.0)

project(robot_arm CXX   
        LANGUAGES CXX C
        VERSION 0.0.0.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/external/cmake)
# include(Arduino-toolchain)

set(EXE_TARGET_NAME robot_arm_executable)

set(${EXE_TARGET_NAME}_HDR
    )
set(${EXE_TARGET_NAME}_SRC
      src/main.cpp
    )


add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} )


# Arduino.h is included in hello_world.cpp, so link with Arduino core
target_link_arduino_libraries(${EXE_TARGET_NAME} PRIVATE core)
# This is needed for the generation of HEX binary and uploading it
target_enable_arduino_upload(${EXE_TARGET_NAME})

# cmake_minimum_required(VERSION 3.0.0)

# project(hello_world CXX)

# add_executable(hello_world hello_world.cpp)

# # Arduino.h is included in hello_world.cpp, so link with Arduino core
# target_link_arduino_libraries(hello_world PRIVATE core)

# # This is needed for the generation of HEX binary and uploading it
# target_enable_arduino_upload(hello_world)
