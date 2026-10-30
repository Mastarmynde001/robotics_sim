cmake_minimum_required(VERSION 3.18)
project(robotics_sim LANGUAGES CXX)

# Force C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable Position Independent Code (-fPIC) for shared libraries
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Default to Release build (-O3 optimization equivalent)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Locate active Python 3 environment and Pybind11 headers
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# Define Pybind11 module target
pybind11_add_module(robotics_sim robotics_sim.cpp)

# Define preprocessor macro used in robotics_sim.cpp
target_compile_definitions(robotics_sim PRIVATE PYBIND11_BUILD)

# Output compiled .so/.pyd directly to project root for seamless Python imports
set_target_properties(robotics_sim PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}"
)