#pragma once

#include "queue.hpp"
#include "actuator.hpp"
#include "state.hpp"
#include <o80/state.hpp>
#include <o80/front_end.hpp>

namespace franka_o80
{
typedef o80::FrontEnd<franka_o80::queue_size, franka_o80::actuator_number, franka_o80::State, o80::VoidExtendedState> FrontEnd;
} // namespace franka_o80

/** @mainpage Welcome to `franka_o80`

Here you will find an [o80](https://github.com/intelligent-soft-robots/o80) `C++` and `Python` wrappers around [libfranka](https://github.com/frankaemika/libfranka).

@tableofcontents

@section Structure
Some files and directories you may need to know about:
 - `include` - include directory for `C++` programmers
 - `src` - directory containing `C++` sources
 - `example` - directory containing finished `franka_o80` projects in form of `C++` sources or `Python` files
 - `build` - `cmake` build directory
 - `build/franka_o80_cpp.so` - shared library for `C++` programmers (could be linked with `-lfranka_o80_cpp`)
 - `build/franka_o80.so` - shared library for `Python` programmers (could be imported with `import franka_o80`)
 - `build/franka_o80_test_*` - dummy libraries for testing purposes
 - `build/example/franka_o80_backend` - executable for starting backend
 - `build/example/franka_o80_control` - application for basic robot control
 - `build/example/franka_o80_selftest` - [Google Test](https://github.com/google/googletest) testing of the library

@section Usage
[o80](https://github.com/intelligent-soft-robots/o80) may be sometimes not intuitive, and `franka_o80` inherits it’s flaws.

The project consists of frontend and backend. Frontend (`o80::Frontend` class) is used by programmer in `C++` or `Python`. It is responsible for sending commands and receiving observations from backend.

Backend (`o80::Backend` and `o80::Standalone` classes) is written in `C++` and does not need to be changed. It is responsible for communication with the [libfranka](https://github.com/frankaemika/libfranka) and needs to be started on the same machine in order to use frontends.

All variables in `franka_o80` are represented as actuators in terms of [o80](https://github.com/intelligent-soft-robots/o80), even control mode, reset, error, velocities, torques, etc., so they are set with `add_command` as all other actuators. When reading observations, all actuators are defined. But some actuators (like `joint_position` and `cartesial_position`) obviously contradict each other, which of them will be used to control the robot, is decided by mode.  Non-intelligent modes directly correspond to `Robot::control` overloaded functions in [libfranka](https://github.com/frankaemika/libfranka). Intelligent modes are implemented with `Robot::control(std::function<Torques(RobotState)>)` function.

@section Dependencies
`franka_o80` depends on:
 - [o80](https://github.com/intelligent-soft-robots/o80)
 - [libfranka](https://github.com/frankaemika/libfranka) (as part of ROS)
 - [pinocchio](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html) (as part of ROS)
 - [Eigen](https://eigen.tuxfamily.org)
 - [Boost](https://www.boost.org) (system and thread)
 - [Google Test](https://github.com/google/googletest)
 - [pybind11](https://github.com/pybind/pybind11) (optionally)
 - [CMake](https://cmake.org) >= `3.10.2`
 - Fully preemptable Linux kernel
 - C++17 compatible compiler

@section Installation
```
mkdir build
cd build
cmake ..
cmake --build .
```

@section Documentation
[Doxygen](https://www.doxygen.nl/manual/starting.html) documentation is provided.
*/