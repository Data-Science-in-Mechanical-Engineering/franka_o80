project(franka_o80)
cmake_minimum_required(VERSION 3.10.2)
set(CMAKE_CXX_STANDARD 17)

# Dependencies #
################

#boost
find_package(Boost REQUIRED COMPONENTS filesystem system thread)
link_libraries(boost_filesystem boost_system boost_thread rt pthread)

#libfranka
include_directories(${libfranka_DIR}/include)
link_directories(${libfranka_DIR}/build)
link_libraries(libfranka.so)

#o80
include_directories(${o80_DIR}/include)
link_directories(${o80_DIR}/build)
link_libraries(real_time_tools_base.so shared_memory_base.so signal_handler_base.so synchronizer_base.so time_series_base.so o80_base.so)

#pybind11
find_package(pybind11)
if(pybind11_FOUND)
	exec_program(python3 ARGS -m pybind11 --includes | cut -d ' ' -f1 | sed 's+-I/+/+g' OUTPUT_VARIABLE pybind11_INCLUDE_DIR1)
	exec_program(python3 ARGS -m pybind11 --includes | cut -d ' ' -f2 | sed 's+-I/+/+g' OUTPUT_VARIABLE pybind11_INCLUDE_DIR2)
endif()
include_directories(${pybind11_INCLUDE_DIR1})
include_directories(${pybind11_INCLUDE_DIR2})

add_compile_definitions(RT_PREEMPT)

# franka_o80_base #
###################

add_library(franka_o80_base SHARED
src/driver.cpp
src/driver_input.cpp
src/driver_output.cpp
src/segment_id.cpp
src/standalone.cpp)

# franka_o80 #
##############

if (DEFINED pybind11_INCLUDE_DIR1)
	add_library(franka_o80 SHARED srcpy/wrappers.cpp)
	target_link_libraries(franka_o80 PRIVATE franka_o80_base)
	set_target_properties(franka_o80 PROPERTIES PREFIX "")
endif()

# franka_o80_backend #
######################

add_executable(franka_o80_backend example/backend.cpp)
target_link_libraries(franka_o80_backend PRIVATE franka_o80_base)

# franka_o80_set_angle_0 #
##########################

add_executable(franka_o80_set_angle_0 example/set_angle_0.cpp)
target_link_libraries(franka_o80_set_angle_0 PRIVATE franka_o80_base)