#################
# Configuretion #
#################

cmake_minimum_required(VERSION 3.14.0)
cmake_policy(SET CMP0048 NEW)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

project(franka_o80 VERSION 1.1.1)

################
# Dependencies #
################

#c++
set(CMAKE_CXX_STANDARD 17)

#libfranka
if (DEFINED franka_DIR)
	set(Franka_DIR ${franka_DIR})
	find_package(Franka REQUIRED)
	set(franka_INCLUDE_DIRS ${Franka_INCLUDE_DIRS})
	set(franka_LIBRARY_DIRS ${franka_DIR})
	set(franka_LIBRARIES franka)
else()
	find_package(catkin REQUIRED COMPONENTS franka_control)
	set(franka_INCLUDE_DIRS ${franka_control_INCLUDE_DIRS})
	set(franka_LIBRARY_DIRS ${franka_control_LIBRARY_DIRS})
	set(franka_LIBRARIES ${franka_control_LIBRARIES})
endif()

#o80
if ((DEFINED o80_DIR) AND (EXISTS ${o80_DIR}/include/o80) AND (EXISTS ${o80_DIR}/lib))
	set(o80 ${o80_DIR})
else()
	find_path(o80 PATHS ${o80_DIR} NAMES include/o80 lib/libo80.so)
endif()
if (NOT "${o80}" MATCHES "o80-NOTFOUND")
	message("-- o80 FOUND. o80 at ${o80}/lib/libo80.so")
	set(o80_INCLUDE_DIRS ${o80}/include)
	set(o80_LIBRARY_DIRS ${o80}/lib)
	set(o80_LIBRARIES real_time_tools shared_memory signal_handler synchronizer time_series o80)
else()
	message(FATAL_ERROR
	"  Could not find an C++ include file \"include/o80/front_end.hpp\"\n"
	"  Make sure \"o80\" is installed or set \"o80_DIR\" to a directory\n"
	"  containing \"include\" and \"lib\" subdirectories of \"o80\" project.")
endif()

#pinocchio
if (DEFINED pinocchio_DIR)
	find_package(pinocchio REQUIRED)
	set(pinocchio_LIBRARY_DIRS ${pinocchio_INCLUDE_DIRS}/../lib)
	set(pinocchio_LIBRARIES pinocchio)
else()
	if(EXISTS $ENV{ROS_PACKAGE_PATH}/cmake/pinocchio)
		set(pinocchio_DIR $ENV{ROS_PACKAGE_PATH}/cmake/pinocchio) #Fix for Ubuntu
	endif()
	find_package(catkin REQUIRED COMPONENTS pinocchio)
endif()

#boost
find_package(Boost REQUIRED COMPONENTS system thread)

#eigen3
find_package(Eigen3 REQUIRED)

#Google Test
find_package(GTest)

#pybind11
find_package(pybind11)
if(pybind11_FOUND)
	exec_program(python3 ARGS -m pybind11 --includes | cut -d ' ' -f1 | sed 's+-I/+/+g' OUTPUT_VARIABLE pybind11_INCLUDE_DIR1)
	exec_program(python3 ARGS -m pybind11 --includes | cut -d ' ' -f2 | sed 's+-I/+/+g' OUTPUT_VARIABLE pybind11_INCLUDE_DIR2)
	exec_program(python3 ARGS --version | cut -d ' ' -f2 | cut -d '.' -f1 OUTPUT_VARIABLE python_VERSION_MAJOR)
	exec_program(python3 ARGS --version | cut -d ' ' -f2 | cut -d '.' -f2 OUTPUT_VARIABLE python_VERSION_MINOR)
	set(pybind11_INCLUDE_DIRS ${pybind11_INCLUDE_DIR1} ${pybind11_INCLUDE_DIR2})
	unset(pybind11_INCLUDE_DIR1)
	unset(pybind11_INCLUDE_DIR2)
endif()

###########
# Library #
###########

# franka_o80
add_library(franka_o80 SHARED
src/driver.cpp
src/driver_input_output.cpp
src/driver_input.cpp
src/driver_output.cpp
src/kinematics.cpp
src/standalone.cpp
src/states.cpp
src/state.cpp)
if (NOT franka_o80_omit_include_directories MATCHES yes)
	target_include_directories(franka_o80 PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
endif()
target_compile_definitions(franka_o80 PUBLIC RT_PREEMPT)
target_compile_definitions(franka_o80 PUBLIC FRANKA_O80_VERSION_MAJOR=${PROJECT_VERSION_MAJOR})
target_compile_definitions(franka_o80 PUBLIC FRANKA_O80_VERSION_MINOR=${PROJECT_VERSION_MINOR})
target_compile_definitions(franka_o80 PUBLIC FRANKA_O80_VERSION_PATCH=${PROJECT_VERSION_PATCH})
target_include_directories(franka_o80 PUBLIC ${franka_INCLUDE_DIRS})
target_include_directories(franka_o80 PUBLIC ${o80_INCLUDE_DIRS})
target_include_directories(franka_o80 PUBLIC ${pinocchio_INCLUDE_DIRS})
target_include_directories(franka_o80 PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_directories(franka_o80 PUBLIC $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}> $<INSTALL_INTERFACE:lib>)
target_link_directories(franka_o80 PUBLIC ${franka_LIBRARY_DIRS})
target_link_directories(franka_o80 PUBLIC ${o80_LIBRARY_DIRS})
target_link_directories(franka_o80 PUBLIC ${pinocchio_LIBRARY_DIRS})
target_link_libraries(franka_o80 PUBLIC ${franka_LIBRARIES})
target_link_libraries(franka_o80 PUBLIC ${o80_LIBRARIES})
target_link_libraries(franka_o80 PUBLIC ${pinocchio_LIBRARIES})
target_link_libraries(franka_o80 PUBLIC boost_system boost_thread rt pthread)
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${PROJECT_SOURCE_DIR}/franka_o80-config-version.cmake" VERSION ${PROJECT_VERSION} COMPATIBILITY AnyNewerVersion)
export(TARGETS franka_o80 FILE "${PROJECT_SOURCE_DIR}/franka_o80-config.cmake")
install(DIRECTORY include/franka_o80 TYPE INCLUDE)
install(TARGETS franka_o80 EXPORT franka_o80-config)
install(EXPORT franka_o80-config DESTINATION lib/cmake/franka_o80)
install(FILES "${PROJECT_SOURCE_DIR}/franka_o80-config-version.cmake" DESTINATION lib/cmake/franka_o80)
install(FILES "${PROJECT_SOURCE_DIR}/model/franka.urdf" DESTINATION share/franka_o80)

# franka_o80_backend
add_executable(franka_o80_backend example/backend.cpp)
target_link_libraries(franka_o80_backend PRIVATE franka_o80)
install(TARGETS franka_o80_backend)

# franka_o80_selftest
if (GTest_FOUND)
	add_executable(franka_o80_selftest example/selftest.cpp)
	target_include_directories(franka_o80_selftest PRIVATE ${GTEST_INCLUDE_DIRS})
	target_link_directories(franka_o80_selftest PRIVATE ${GTEST_LIBRARY_DIRS})
	target_link_libraries(franka_o80_selftest PRIVATE franka_o80)
	target_link_libraries(franka_o80_selftest PRIVATE ${GTEST_LIBRARIES})
	install(TARGETS franka_o80_selftest)
endif()

# franka_o80_python
if (pybind11_FOUND)
	add_library(franka_o80_python SHARED src/wrappers.cpp)
	if (franka_o80_omit_include_directories MATCHES yes)
		target_compile_definitions(franka_o80_python PRIVATE FRANKA_O80_WRAPPERS_INSTALLED)
	endif()
	target_include_directories(franka_o80_python PRIVATE ${pybind11_INCLUDE_DIRS})
	target_link_libraries(franka_o80_python PRIVATE franka_o80)
	set_target_properties(franka_o80_python PROPERTIES PREFIX "")
	set_target_properties(franka_o80_python PROPERTIES OUTPUT_NAME "franka_o80.cpython-${python_VERSION_MAJOR}${python_VERSION_MINOR}-${CMAKE_LIBRARY_ARCHITECTURE}")
	install(TARGETS franka_o80_python DESTINATION "lib/python${python_VERSION_MAJOR}.${python_VERSION_MINOR}/dist-packages")
endif()

# franka_o80_control
add_executable(franka_o80_control example/control.cpp)
target_link_libraries(franka_o80_control PUBLIC franka_o80)
install(TARGETS franka_o80_control)

# franka_o80_control.py
configure_file(example/control.py franka_o80_control.py)
install(PROGRAMS example/control.py RENAME franka_o80_control.py TYPE BIN)

# franka_o80_control_trajectory
configure_file(example/trajectory franka_o80_control_trajectory)
