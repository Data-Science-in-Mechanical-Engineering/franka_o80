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

##############
# REAL ROBOT #
##############

# franka_o80_base #
###################

add_library(franka_o80_base SHARED
src/driver.cpp
src/driver_input_output.cpp
src/driver_input.cpp
src/driver_output.cpp
src/segment_id.cpp
src/standalone.cpp
src/state.cpp)

target_include_directories(franka_o80_base PUBLIC ${o80_DIR}/include)
target_include_directories(franka_o80_base PUBLIC ${libfranka_DIR}/include)
target_link_directories(franka_o80_base PUBLIC ${libfranka_DIR}/build)
target_link_directories(franka_o80_base PUBLIC ${o80_DIR}/build)
target_link_libraries(franka_o80_base PUBLIC boost_filesystem boost_system boost_thread rt pthread)
target_link_libraries(franka_o80_base PUBLIC libfranka.so)
target_link_libraries(franka_o80_base PUBLIC real_time_tools_base.so shared_memory_base.so signal_handler_base.so synchronizer_base.so time_series_base.so o80_base.so)

# franka_o80 #
##############

if (DEFINED pybind11_INCLUDE_DIR1)
	add_library(franka_o80 SHARED src/wrappers.cpp)
	target_include_directories(franka_o80 PUBLIC ${pybind11_INCLUDE_DIR1})
	target_include_directories(franka_o80 PUBLIC ${pybind11_INCLUDE_DIR2})
	target_link_libraries(franka_o80 PUBLIC franka_o80_base)
	set_target_properties(franka_o80 PROPERTIES PREFIX "")
endif()

# franka_o80_backend #
######################

add_executable(franka_o80_backend example/backend.cpp)
target_link_libraries(franka_o80_backend PUBLIC franka_o80_base)

# franka_o80_set #
##################

add_executable(franka_o80_set example/set.cpp)
target_link_libraries(franka_o80_set PUBLIC franka_o80_base)

# franka_o80_get #
##################

add_executable(franka_o80_get example/get.cpp)
target_link_libraries(franka_o80_get PUBLIC franka_o80_base)

# franka_o80_selftest #
#######################

add_executable(franka_o80_selftest example/selftest.cpp)
target_link_libraries(franka_o80_selftest PUBLIC franka_o80_base)
target_link_libraries(franka_o80_selftest PUBLIC gtest)

###########
# TESTING #
###########

# franka_o80_base_test #
########################

add_library(franka_o80_base_test SHARED
src/driver.cpp
src/driver_input_output.cpp
src/driver_input.cpp
src/driver_output.cpp
src/segment_id.cpp
src/standalone.cpp
src/state.cpp
src/franka_test.cpp)

target_compile_definitions(franka_o80_base_test PUBLIC FRANKA_O80_TEST)
target_compile_definitions(franka_o80_base_test PUBLIC FRANKA_O80_DEBUG)
target_include_directories(franka_o80_base_test PUBLIC ${o80_DIR}/include)
target_link_directories(franka_o80_base_test PUBLIC ${o80_DIR}/build)
target_link_libraries(franka_o80_base_test PUBLIC boost_filesystem boost_system boost_thread rt pthread)
target_link_libraries(franka_o80_base_test PUBLIC real_time_tools_base.so shared_memory_base.so signal_handler_base.so synchronizer_base.so time_series_base.so o80_base.so)

# franka_o80 #
##############

if (DEFINED pybind11_INCLUDE_DIR1)
	add_library(franka_o80_test SHARED src/wrappers.cpp)
	target_include_directories(franka_o80_test PUBLIC ${pybind11_INCLUDE_DIR1})
	target_include_directories(franka_o80_test PUBLIC ${pybind11_INCLUDE_DIR2})
	target_link_libraries(franka_o80_test PUBLIC franka_o80_base_test)
	set_target_properties(franka_o80_test PROPERTIES PREFIX "")
endif()

# franka_o80_backend #
######################

add_executable(franka_o80_backend_test example/backend.cpp)
target_link_libraries(franka_o80_backend_test PUBLIC franka_o80_base_test)

# franka_o80_set #
##################

add_executable(franka_o80_set_test example/set.cpp)
target_link_libraries(franka_o80_set_test PUBLIC franka_o80_base_test)

# franka_o80_get #
##################

add_executable(franka_o80_get_test example/get.cpp)
target_link_libraries(franka_o80_get_test PUBLIC franka_o80_base_test)

# franka_o80_selftest #
#######################

add_executable(franka_o80_selftest_test example/selftest.cpp)
target_link_libraries(franka_o80_selftest_test PUBLIC franka_o80_base_test)
target_compile_definitions(franka_o80_selftest_test PUBLIC FRANKA_O80_TEST)
target_link_libraries(franka_o80_selftest_test PUBLIC gtest)