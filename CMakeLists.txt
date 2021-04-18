project(franka_o80)
cmake_minimum_required(VERSION 3.10.2)
set(CMAKE_CXX_STANDARD 17)

include_directories(${libfranka_DIR}/include)
include_directories(${o80_DIR}/include)
add_compile_definitions(RT_PREEMPT)

# Dependencies #
################

find_package(pybind11)
if(pybind11_FOUND)
	exec_program(python3 ARGS -m pybind11 --includes | cut -d ' ' -f1 | sed 's+-I/+/+g' OUTPUT_VARIABLE pybind11_INCLUDE_DIR1)
	exec_program(python3 ARGS -m pybind11 --includes | cut -d ' ' -f2 | sed 's+-I/+/+g' OUTPUT_VARIABLE pybind11_INCLUDE_DIR2)
endif()
include_directories(${pybind11_INCLUDE_DIR1})
include_directories(${pybind11_INCLUDE_DIR2})

# franka_o80_base #
###################

add_library(franka_o80_base SHARED
src/driver.cpp
src/driver_in.cpp
src/driver_out.cpp
src/standalone.cpp)

# franka_o80 #
##############

if (DEFINED pybind11_INCLUDE_DIR1)
	add_library(franka_o80 SHARED src/wrappers/wrappers.cpp)
	target_link_libraries(franka_o80 PRIVATE franka_o80_base)
endif()
