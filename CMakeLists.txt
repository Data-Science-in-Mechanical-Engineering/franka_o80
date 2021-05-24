project(franka_o80)

################
# Dependencies #
################

#cmake
cmake_minimum_required(VERSION 3.10.2)

#c++
set(CMAKE_CXX_STANDARD 17)

#boost
find_package(Boost REQUIRED COMPONENTS filesystem system thread)

#pybind11
find_package(pybind11)
if(pybind11_FOUND)
	exec_program(python3 ARGS -m pybind11 --includes | cut -d ' ' -f1 | sed 's+-I/+/+g' OUTPUT_VARIABLE pybind11_INCLUDE_DIR1)
	exec_program(python3 ARGS -m pybind11 --includes | cut -d ' ' -f2 | sed 's+-I/+/+g' OUTPUT_VARIABLE pybind11_INCLUDE_DIR2)
endif()

#PREEMPT-RT
add_compile_definitions(RT_PREEMPT)

#libfranka
find_package(catkin REQUIRED COMPONENTS roscpp)
exec_program(echo ARGS ${catkin_INCLUDE_DIRS} | cut -d ' ' -f1 | sed 's+$+/libfranka+g' OUTPUT_VARIABLE libfranka_INCLUDE_DIR1)
exec_program(echo ARGS ${catkin_INCLUDE_DIRS} | cut -d ' ' -f2 | sed 's+$+/libfranka+g' OUTPUT_VARIABLE libfranka_INCLUDE_DIR2)

##############
# REAL ROBOT #
##############

# libfranka_o80 #
#################

add_library(libfranka_o80 SHARED
src/driver.cpp
src/driver_input_output.cpp
src/driver_input.cpp
src/driver_output.cpp
src/segment_id.cpp
src/standalone.cpp
src/state.cpp)

target_include_directories(libfranka_o80 PUBLIC ${libfranka_INCLUDE_DIR1})
target_include_directories(libfranka_o80 PUBLIC ${libfranka_INCLUDE_DIR2})
target_link_directories(libfranka_o80 PUBLIC ${catkin_LIBRARY_DIRS})
target_link_libraries(libfranka_o80 PUBLIC boost_filesystem boost_system boost_thread rt pthread)
target_link_libraries(libfranka_o80 PUBLIC franka)
target_link_libraries(libfranka_o80 PUBLIC real_time_tools shared_memory signal_handler synchronizer time_series o80)
set_target_properties(libfranka_o80 PROPERTIES PREFIX "")

# franka_o80 #
##############

if (DEFINED pybind11_INCLUDE_DIR1)
	add_library(franka_o80 SHARED src/wrappers.cpp)
	target_include_directories(franka_o80 PUBLIC ${pybind11_INCLUDE_DIR1})
	target_include_directories(franka_o80 PUBLIC ${pybind11_INCLUDE_DIR2})
	target_link_libraries(franka_o80 PUBLIC libfranka_o80)
	set_target_properties(franka_o80 PROPERTIES PREFIX "")
endif()

# franka_o80_backend #
######################

add_executable(franka_o80_backend example/backend.cpp)
target_link_libraries(franka_o80_backend PUBLIC libfranka_o80)

# franka_o80_set #
##################

add_executable(franka_o80_set example/set.cpp)
target_link_libraries(franka_o80_set PUBLIC libfranka_o80)

# franka_o80_get #
##################

add_executable(franka_o80_get example/get.cpp)
target_link_libraries(franka_o80_get PUBLIC libfranka_o80)

# franka_o80_selftest #
#######################

add_executable(franka_o80_selftest example/selftest.cpp)
target_link_libraries(franka_o80_selftest PUBLIC libfranka_o80)
target_link_libraries(franka_o80_selftest PUBLIC gtest)

###########
# TESTING #
###########

# franka_o80_test #
###################

add_library(libfranka_o80_test SHARED
src/driver.cpp
src/driver_input_output.cpp
src/driver_input.cpp
src/driver_output.cpp
src/segment_id.cpp
src/standalone.cpp
src/state.cpp
src/franka_test.cpp)

target_compile_definitions(libfranka_o80_test PUBLIC FRANKA_O80_TEST)
target_compile_definitions(libfranka_o80_test PUBLIC FRANKA_O80_DEBUG)
target_link_libraries(libfranka_o80_test PUBLIC boost_filesystem boost_system boost_thread rt pthread)
target_link_libraries(libfranka_o80_test PUBLIC real_time_tools shared_memory signal_handler synchronizer time_series o80)
set_target_properties(libfranka_o80_test PROPERTIES PREFIX "")

# franka_o80_test #
###################

if (DEFINED pybind11_INCLUDE_DIR1)
	add_library(franka_o80_test SHARED src/wrappers.cpp)
	target_include_directories(franka_o80_test PUBLIC ${pybind11_INCLUDE_DIR1})
	target_include_directories(franka_o80_test PUBLIC ${pybind11_INCLUDE_DIR2})
	target_link_libraries(franka_o80_test PUBLIC libfranka_o80_test)
	set_target_properties(franka_o80_test PROPERTIES PREFIX "")
endif()

# franka_o80_backend_test #
###########################

add_executable(franka_o80_backend_test example/backend.cpp)
target_link_libraries(franka_o80_backend_test PUBLIC libfranka_o80_test)

# franka_o80_set_test #
#######################

add_executable(franka_o80_set_test example/set.cpp)
target_link_libraries(franka_o80_set_test PUBLIC libfranka_o80_test)

# franka_o80_get_test #
#######################

add_executable(franka_o80_get_test example/get.cpp)
target_link_libraries(franka_o80_get_test PUBLIC libfranka_o80_test)

# franka_o80_selftest_test #
############################

add_executable(franka_o80_selftest_test example/selftest.cpp)
target_link_libraries(franka_o80_selftest_test PUBLIC libfranka_o80_test)
target_compile_definitions(franka_o80_selftest_test PUBLIC FRANKA_O80_TEST)
target_link_libraries(franka_o80_selftest_test PUBLIC gtest)