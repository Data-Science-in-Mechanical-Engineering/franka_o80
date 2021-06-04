project(franka_o80)

################
# Dependencies #
################

#cmake
cmake_minimum_required(VERSION 3.10.2)

#c++
set(CMAKE_CXX_STANDARD 17)

#real-time
add_compile_definitions(RT_PREEMPT)

#boost
find_package(Boost REQUIRED COMPONENTS system thread)

#eigen3
find_package(Eigen3 REQUIRED)

#pybind11
find_package(pybind11)
if(pybind11_FOUND)
	exec_program(python3 ARGS -m pybind11 --includes | cut -d ' ' -f1 | sed 's+-I/+/+g' OUTPUT_VARIABLE pybind11_INCLUDE_DIR1)
	exec_program(python3 ARGS -m pybind11 --includes | cut -d ' ' -f2 | sed 's+-I/+/+g' OUTPUT_VARIABLE pybind11_INCLUDE_DIR2)
endif()

#pinocchio
find_package(catkin REQUIRED COMPONENTS pinocchio)

#moveit
find_package(catkin REQUIRED COMPONENTS moveit_ros_planning_interface roscpp)

#libfranka
find_package(catkin REQUIRED COMPONENTS franka_control)

#################
# Testing robot #
#################

# franka_o80_test_cpp
add_library(franka_o80_test_cpp SHARED
src/cartesial_states.cpp
src/driver.cpp
src/driver_input_output.cpp
src/driver_input.cpp
src/driver_output.cpp
src/franka_test.cpp
src/standalone.cpp
src/states.cpp
src/state.cpp)

target_compile_definitions(franka_o80_test_cpp PUBLIC FRANKA_O80_TEST)

target_include_directories(franka_o80_test_cpp PUBLIC ${moveit_ros_planning_interface_INCLUDE_DIRS})
target_include_directories(franka_o80_test_cpp PUBLIC ${pinocchio_INCLUDE_DIRS})
target_include_directories(franka_o80_test_cpp PUBLIC ${EIGEN3_INCLUDE_DIRS})

target_link_directories(franka_o80_test_cpp PUBLIC ${moveit_ros_planning_interface_LIBRARY_DIRS})

target_link_libraries(franka_o80_test_cpp PUBLIC ${moveit_ros_planning_interface_LIBRARIES})
target_link_libraries(franka_o80_test_cpp PUBLIC pinocchio)
target_link_libraries(franka_o80_test_cpp PUBLIC boost_system boost_thread rt pthread)
target_link_libraries(franka_o80_test_cpp PUBLIC real_time_tools shared_memory signal_handler synchronizer time_series o80)

# franka_o80_test
if (DEFINED pybind11_INCLUDE_DIR1)
	add_library(franka_o80_test SHARED src/wrappers.cpp)
	target_include_directories(franka_o80_test PUBLIC ${pybind11_INCLUDE_DIR1})
	target_include_directories(franka_o80_test PUBLIC ${pybind11_INCLUDE_DIR2})
	target_link_libraries(franka_o80_test PUBLIC franka_o80_test_cpp)
	set_target_properties(franka_o80_test PROPERTIES PREFIX "")
endif()

# franka_o80_backend
add_executable(franka_o80_test_backend example/backend.cpp)
target_link_libraries(franka_o80_test_backend PUBLIC franka_o80_test_cpp)

# franka_o80_control
add_executable(franka_o80_test_control example/control.cpp)
target_link_libraries(franka_o80_test_control PUBLIC franka_o80_test_cpp)

# franka_o80_selftest
add_executable(franka_o80_test_selftest example/selftest.cpp)
target_link_libraries(franka_o80_test_selftest PUBLIC franka_o80_test_cpp)
target_link_libraries(franka_o80_test_selftest PUBLIC gtest)

##############
# Real robot #
##############

# franka_o80_cpp
add_library(franka_o80_cpp SHARED
src/cartesial_states.cpp
src/driver.cpp
src/driver_input_output.cpp
src/driver_input.cpp
src/driver_output.cpp
src/standalone.cpp
src/states.cpp
src/state.cpp)

target_include_directories(franka_o80_cpp PUBLIC ${franka_control_INCLUDE_DIRS})
target_include_directories(franka_o80_cpp PUBLIC ${pinocchio_INCLUDE_DIRS})
target_include_directories(franka_o80_cpp PUBLIC ${EIGEN3_INCLUDE_DIRS})

target_link_directories(franka_o80_cpp PUBLIC ${franka_control_LIBRARY_DIRS})

target_link_libraries(franka_o80_cpp PUBLIC franka)
target_link_libraries(franka_o80_cpp PUBLIC pinocchio)
target_link_libraries(franka_o80_cpp PUBLIC boost_system boost_thread rt pthread)
target_link_libraries(franka_o80_cpp PUBLIC real_time_tools shared_memory signal_handler synchronizer time_series o80)

# franka_o80
if (DEFINED pybind11_INCLUDE_DIR1)
	add_library(franka_o80 SHARED src/wrappers.cpp)
	target_include_directories(franka_o80 PUBLIC ${pybind11_INCLUDE_DIR1})
	target_include_directories(franka_o80 PUBLIC ${pybind11_INCLUDE_DIR2})
	target_link_libraries(franka_o80 PUBLIC franka_o80_cpp)
	set_target_properties(franka_o80 PROPERTIES PREFIX "")
endif()

# franka_o80_backend
add_executable(franka_o80_backend example/backend.cpp)
target_link_libraries(franka_o80_backend PUBLIC franka_o80_cpp)

# franka_o80_control
add_executable(franka_o80_control example/control.cpp)
target_link_libraries(franka_o80_control PUBLIC franka_o80_cpp)

# franka_o80_selftest
add_executable(franka_o80_selftest example/selftest.cpp)
target_link_libraries(franka_o80_selftest PUBLIC franka_o80_cpp)
target_link_libraries(franka_o80_selftest PUBLIC gtest)