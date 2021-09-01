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

/** @mainpage Welcome to `franka_o80`!
Here you will find a library for control of [Franka Emika Panda](https://www.franka.de/) robot. The library is a specialization of [o80](https://github.com/intelligent-soft-robots/o80) templates and is based on [libfranka](https://github.com/frankaemika/libfranka). It includes both `C++` headers and `Python` bindings.

@tableofcontents

@section Example
Firstly, you need to start a backend with the following command (`ID` and `IP` are placeholders):
```
./franka_o80_backend ${ID} ${IP} &
```
Then you will be able to run a `C++` program:
```
#include <franka_o80/front_end.hpp>
int main()
{
    franka_o80::FrontEnd frontend("ID");
    frontend.add_command(franka_o80::control_mode, franka_o80::Mode::intelligent_position, o80::Mode::QUEUE);
    frontend.add_command(franka_o80::joint_position[0], 1.0, o80::Duration_us::seconds(5), o80::Mode::QUEUE);
    frontend.pulse_and_wait();
}
```
...or a correspondent `Python` script:
```
import o80
import franka_o80
frontend = franka_o80.FrontEnd("ID")
frontend.add_command(franka_o80.control_mode(), franka_o80.State(franka_o80.Mode.intelligent_position), o80.Mode.QUEUE)
frontend.add_command(franka_o80.joint_position(0), franka_o80.State(1.0), o80.Duration_us.seconds(5), o80.Mode.QUEUE)
frontend.pulse_and_wait()
```

@section Structure
Some files and directories you may need to know about:
 - `include` - include directory for `C++` programmers
 - `src` - directory containing `C++` sources
 - `example` - directory containing finished `franka_o80` projects in form of `C++` sources or `Python` files
 - `build` - default name for `cmake` build directory
 - `build/libfranka_o80_cpp.so` - shared library for `C++` programmers (could be linked with `-lfranka_o80_cpp`)
 - `build/franka_o80.so` - shared library for `Python` programmers (could be imported with `import franka_o80`)
 - `build/franka_o80_backend` - executable for backend control
 - `build/franka_o80_selftest` - executable for testing the library with [Google Test](https://github.com/google/googletest)
 - `build/franka_o80_control` - example executable for robot control
 - `build/franka_o80_control.py` - example `Python` script for robot control
 - `build/franka_o80_control_trajectory` - example of commands that can be executed with `franka_o80_control`

@section Requirements
`franka_o80` requires:
 - [o80](https://github.com/intelligent-soft-robots/o80)
 - [libfranka](https://github.com/frankaemika/libfranka) (as part of `ROS`)
 - [pinocchio](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html) (as part of `ROS`)
 - [Eigen](https://eigen.tuxfamily.org)
 - [Boost](https://www.boost.org) (`system` and `thread`)
 - [Google Test](https://github.com/google/googletest) (optionally)
 - [pybind11](https://github.com/pybind/pybind11) (optionally)
 - [CMake](https://cmake.org) >= `3.10.2`
 - Fully preemptable Linux kernel
 - C++17 compatible compiler
 - `root` privileges

@section Building
`franka_o80` can be built with [CMake](https://cmake.org) using following commands:
```
mkdir build
cd build
cmake ..
cmake --build .
```

@section Documentation
`Python` docstrings are provided. `C++` code is documented with comments. [Doxygen](https://www.doxygen.nl) documentation may be generated with `doxygen` command. Example projects print human-readable help messages.

@section Notes
`franka_o80` may sometimes be non-intuitive. So here are some important notes:
 - `franka_o80` is a specialization of some [o80](https://github.com/intelligent-soft-robots/o80) templates. Some vital general functions may be implemented and documented in [o80](https://github.com/intelligent-soft-robots/o80), not `franka_o80`.
 - The project consists of frontend and backend. **Frontend** is responsible for sending commands and receiving observations from backend. `franka_o80::Frontend` class is the main class to be used by programmers in `C++` or `Python`. **Backend** is responsible for communication with the [libfranka](https://github.com/frankaemika/libfranka) and needs to be started on the machine in order to use frontends. This could be done with `franka_o80_backend` executable or with some functions, like `franka_o80::start_standalone`.
 - All variables in `franka_o80` are implemented as actuators in terms of [o80](https://github.com/intelligent-soft-robots/o80), even control mode, reset signal, error, velocities, torques, etc., and they are controlled with `Frontend::add_command` like real actuators. When reading observations, all actuators are defined. But some of them (like `franka_o80::joint_position` and `franka_o80::cartesian_position`) obviously contradict each other, and which of them will be used to control the robot, is decided by `control_mode` actuator.
 - All actuators, like `franka_o80::control_mode` or `franka_o80::joint_position`, are just constant numbers. They are only useful in `Frontend::add_command`, `States::get` and `States::set` functions.
 - Frontend does not throw exceptions when an error in backend occurs. The only way to know about backend errors is to read `franka_o80::control_error` actuator.
 - All `Frontend::add_command` functions accept `franka_o80::State`. This is why the class encapsulates both real numbers, quaternions, mode and error enumerations, and some dynamic typization is done in the class (even in `C++`). Backend will, for example, expect the state applied to `control_mode` actuator to contain `franka_o80::Mode` value, but frontend does not check state types and does not throw exceptions. The only way to know that `Frontend::add_command` was called with wrong state type is to read `franka_o80::control_error` actuator.
 - The gripper is controlled with `franka_o80::gripper_grasped`, and may only be in "opened" or "closed" state. No finger width control is currently possible.

@section Contributors
 - Kyrylo Sovailo
 - Original [o80](https://github.com/intelligent-soft-robots/o80) is authored by Vincent Berenz
 - Models from `model` directory were generated with [MoveIt Setup Assistant](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) from `ROS` package `franka_description`
*/