#include <o80/pybind11_helper.hpp>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#ifdef FRANKA_O80_WRAPPERS_INSTALLED
    #include <franka_o80/kinematics.hpp>
    #include <franka_o80/standalone.hpp>
    #include <franka_o80/limits.hpp>
    #include <franka_o80/actuator.hpp>
    #include <franka_o80/version.hpp>
#else
    #include "../include/franka_o80/kinematics.hpp"
    #include "../include/franka_o80/standalone.hpp"
    #include "../include/franka_o80/limits.hpp"
    #include "../include/franka_o80/actuator.hpp"
    #include "../include/franka_o80/version.hpp"
#endif
#include <stdexcept>

PYBIND11_MODULE(franka_o80, m)
{
    m.doc() = "franka_o80 " + std::to_string(franka_o80::version_major) + "." + std::to_string(franka_o80::version_minor) + "." + std::to_string(franka_o80::version_patch) +
    " is a library for control of Franka Emika Panda robot. The library is a specialization of o80 templates and is based on libfranka";
    o80::create_python_bindings<franka_o80::Standalone, o80::NO_STATE>(m);

    //actuator.hpp
    m.def("actuator_number",        []()      -> int { return franka_o80::actuator_number; },                                                                                           "Actuator number");
    m.def("control_mode",           []()      -> int { return franka_o80::control_mode; },                                                                                              "Actuator number corresponding to control mode. Contains `franka_o80::Mode` values");
    m.def("control_error",          []()      -> int { return franka_o80::control_error; },                                                                                             "Actuator number corresponding to error indicator. Contains `franka_o80::Error` values");
    m.def("control_reset",          []()      -> int { return franka_o80::control_reset; },                                                                                             "Actuator numbers corresponding to reset");
    m.def("gripper_width",          []()      -> int { return franka_o80::gripper_width; },                                                                                             "Actuator number corresponding to gripper width");
    m.def("gripper_temperature",    []()      -> int { return franka_o80::gripper_temperature; },                                                                                       "Actuator number corresponding to gripper temperature");
    m.def("gripper_force",          []()      -> int { return franka_o80::gripper_force; },                                                                                             "Actuator number corresponding to gripper force");
    m.def("joint_position",         [](int i) -> int { if (i < 0 || i > 6) throw std::range_error("franka_o80 invalid joint index"); return franka_o80::joint_position[i]; },           "Actuator numbers corresponding to robot angular positions");
    m.def("joint_velocity",         [](int i) -> int { if (i < 0 || i > 6) throw std::range_error("franka_o80 invalid joint index"); return franka_o80::joint_velocity[i]; },           "Actuator numbers corresponding to robot angular velocities");
    m.def("joint_torque",           [](int i) -> int { if (i < 0 || i > 6) throw std::range_error("franka_o80 invalid joint index"); return franka_o80::joint_torque[i]; },             "Actuator numbers corresponding to robot torques");
    m.def("cartesian_position",     [](int i) -> int { if (i < 0 || i > 3) throw std::range_error("franka_o80 invalid dimension index"); return franka_o80::cartesian_position[i]; },   "Actuator numbers corresponding to effector position");
    m.def("cartesian_orientation",  []()      -> int { return franka_o80::cartesian_orientation; },                                                                                     "Actuator numbers corresponding to effector orientation. Contains quaternion values");
    m.def("cartesian_velocity",     [](int i) -> int { if (i < 0 || i > 3) throw std::range_error("franka_o80 invalid dimension index"); return franka_o80::cartesian_velocity[i]; },   "Actuator numbers corresponding to effector translation velocity");
    m.def("cartesian_rotation",     [](int i) -> int { if (i < 0 || i > 3) throw std::range_error("franka_o80 invalid dimension index"); return franka_o80::cartesian_rotation[i]; },   "Actuator numbers corresponding to effector rotation velocity (WRT)");
    m.def("joint_stiffness",        [](int i) -> int { if (i < 0 || i > 7) throw std::range_error("franka_o80 invalid dimension index"); return franka_o80::joint_stiffness[i]; },      "Actuator numbers corresponding to joint-space stiffness");
    m.def("joint_damping",          [](int i) -> int { if (i < 0 || i > 7) throw std::range_error("franka_o80 invalid dimension index"); return franka_o80::joint_damping[i]; },        "Actuator numbers corresponding to joint-space damping");
    m.def("cartesian_stiffness",    [](int i) -> int { if (i < 0 || i > 6) throw std::range_error("franka_o80 invalid dimension index"); return franka_o80::cartesian_stiffness[i]; },  "Actuator numbers corresponding to cartesian stiffness (for velocity and rotation)");
    m.def("cartesian_damping",      [](int i) -> int { if (i < 0 || i > 6) throw std::range_error("franka_o80 invalid dimension index"); return franka_o80::cartesian_damping[i]; },    "Actuator numbers corresponding to cartesian damping (for velocity and rotation)");
    
    //error.hpp
    pybind11::enum_<franka_o80::Error>(m, "Error",                                                          "Enumeration of backend error indicators")
    .value("ok",                                    franka_o80::Error::ok,                                  "No errors")
    .value("robot_command_exception",               franka_o80::Error::robot_command_exception,             "franka::Robot has thrown franka::CommandException")
    .value("robot_control_exception",               franka_o80::Error::robot_control_exception,             "franka::Robot has thrown franka::ControlException")
    .value("robot_invalid_operation_exception",     franka_o80::Error::robot_invalid_operation_exception,   "franka::Robot has thrown franka::InvalidOperationException")
    .value("robot_network_exception",               franka_o80::Error::robot_network_exception,             "franka::Robot has thrown franka::NetworkException")
    .value("robot_realtime_exception",              franka_o80::Error::robot_realtime_exception,            "franka::Robot has thrown franka::RealtimeException")
    .value("robot_invalid_argument_exception",      franka_o80::Error::robot_invalid_argument_exception,    "franka::Robot has thrown std::invalid_argument exception")
    .value("robot_other_exception",                 franka_o80::Error::robot_other_exception,               "franka::Robot has thrown other exception")
    .value("gripper_command_exception",             franka_o80::Error::gripper_command_exception,           "franka::Gripper has thrown franka::CommandException")
    .value("gripper_network_exception",             franka_o80::Error::gripper_network_exception,           "franka::Gripper has thrown franka::NetworkException")
    .value("gripper_invalid_operation_exception",   franka_o80::Error::gripper_invalid_operation_exception, "franka::Gripper has thrown franka::InvalidOperationException")
    .value("gripper_other_exception",               franka_o80::Error::gripper_other_exception,             "franka::Gripper has thrown other exception");

    //kinematics.hpp
    m.def("joint_to_cartesian", &franka_o80::joint_to_cartesian,                                                                            "Transforms joint positions to cartesian position and orientation",                         pybind11::arg("states"));
    m.def("cartesian_to_joint", pybind11::overload_cast<franka_o80::States&>(&franka_o80::cartesian_to_joint),                              "Transforms cartesian position and orientation to joint position",                          pybind11::arg("states"));
    m.def("cartesian_to_joint", pybind11::overload_cast<franka_o80::States&, const franka_o80::States&>(&franka_o80::cartesian_to_joint),   "Transforms cartesian position and orientation to joint position wit hcustom initial guess",pybind11::arg("states"), pybind11::arg("hint"));

    //limits.hpp
    m.def("joint_position_min",     [](int i) -> double { if (i < 0 || i > 6) throw std::range_error("franka_o80 invalid joint index"); return franka_o80::joint_position_min[i]; },     "Robot minimal positions for each joint");
    m.def("joint_position_max",     [](int i) -> double { if (i < 0 || i > 6) throw std::range_error("franka_o80 invalid joint index"); return franka_o80::joint_position_max[i]; },     "Robot maximal positions for each joint");
    m.def("joint_velocity_max",     [](int i) -> double { if (i < 0 || i > 6) throw std::range_error("franka_o80 invalid joint index"); return franka_o80::joint_velocity_max[i]; },     "Robot maximal angular velocities for each joint");
    m.def("joint_acceleration_max", [](int i) -> double { if (i < 0 || i > 6) throw std::range_error("franka_o80 invalid joint index"); return franka_o80::joint_acceleration_max[i]; }, "Robot maximal angular accelerations for each joint");
    m.def("joint_jerk_max",         [](int i) -> double { if (i < 0 || i > 6) throw std::range_error("franka_o80 invalid joint index"); return franka_o80::joint_jerk_max[i]; },         "Robot maximal angular jerks for each joint");
    m.def("joint_torque_max",       [](int i) -> double { if (i < 0 || i > 6) throw std::range_error("franka_o80 invalid joint index"); return franka_o80::joint_torque_max[i]; },       "Robot maximal torques for each joint");
    m.def("joint_dtorque_max",      [](int i) -> double { if (i < 0 || i > 6) throw std::range_error("franka_o80 invalid joint index"); return franka_o80::joint_dtorque_max[i]; },      "Robot maximal derivaties of torques for each joint");

    //mode.hpp
    pybind11::enum_<franka_o80::Mode>(m, "Mode",                                                "Enumeration of control modes. Mode defines which actuators does the backend listen and which does ignore")
    .value("invalid",                       franka_o80::Mode::invalid,                          "Backend listens only mode and reset, robot does not move")
    .value("torque",                        franka_o80::Mode::torque,                           "Backend listens torques")
    .value("torque_position",               franka_o80::Mode::torque_position,                  "Backend listens torques and joint positions")
    .value("torque_velocity",               franka_o80::Mode::torque_velocity,                  "Backend listens torques and joint velocities")
    .value("torque_cartesian_position",     franka_o80::Mode::torque_cartesian_position,        "Backend listens torques and cartesian position")
    .value("torque_cartesian_velocity",     franka_o80::Mode::torque_cartesian_velocity,        "Backend listens torques and cartesian velocities")
    .value("position",                      franka_o80::Mode::position,                         "Backend listens joint positions")
    .value("velocity",                      franka_o80::Mode::velocity,                         "Backend listens joint velocities")
    .value("cartesian_position",            franka_o80::Mode::cartesian_position,               "Backend listens cartesian positions")
    .value("cartesian_velocity",            franka_o80::Mode::cartesian_velocity,               "Backend listens cartesian velocities")
    .value("intelligent_position",          franka_o80::Mode::intelligent_position,             "Backend listens joint positions, but calculates torques itself. In practice, the robot moves smoothly im this mode")
    .value("intelligent_cartesian_position",franka_o80::Mode::intelligent_cartesian_position,   "Backend listens cartesian positions, but calculates torques itself. In practice, the robot moves smoothly im this mode");

    //queue.hpp
    m.def("actuator_number",    []() -> int { return franka_o80::queue_size; }, "Command queue size");

    //standalone.hpp
    m.def("start_standalone",       &franka_o80::start_standalone,      "Starts standalone",                    pybind11::arg("segment_id"), pybind11::arg("ip"));
    m.def("standalone_is_running",  &franka_o80::standalone_is_running, "Checks whether standalone is running", pybind11::arg("segment_id"));
    m.def("please_stop",            &franka_o80::please_stop,           "Gently stops standalone",              pybind11::arg("segment_id"));
    m.def("stop_standalone",        &franka_o80::stop_standalone,       "Stops standalone",                     pybind11::arg("segment_id"));

    //state.hpp
    pybind11::enum_<franka_o80::State::Type>(
        pybind11::class_<franka_o80::State> (m, "State",            "Actuator state. Some dynamic typization is done within the class, so it may contain real number, quaternion, franka_o80.Mode or franka_o80.Error")
        .def(pybind11::init<>(),                                    "Creates state with zero real value")
        .def(pybind11::init<double>(),                              "Creates state with given real value")
        .def(pybind11::init<const Eigen::Quaterniond&>(),           "Creates state with given quaternion value")
        .def(pybind11::init<const Eigen::Matrix<double, 4, 1>&>(),  "Creates state with given quaternion value given as wxyz")
        .def(pybind11::init<const Eigen::Matrix<double, 3, 1>&>(),  "Creates state with given quaternion value given as Euler angles")
        .def(pybind11::init<franka_o80::Mode>(),                    "Creates state with given mode value")
        .def(pybind11::init<franka_o80::Error>(),                   "Creates state with given error value")
        .def(pybind11::init<const franka_o80::State&>(),            "Copies state")
        .def("set_real",        &franka_o80::State::set_real,       "Sets state's value to real value")
        .def("set_quaternion",  &franka_o80::State::set_quaternion, "Sets state's value to quaternion value")
        .def("set_wxyz",        &franka_o80::State::set_wxyz,       "Sets state's value to quaternion value given as wxyz")
        .def("set_euler",       &franka_o80::State::set_euler,      "Sets state's value to quaternion value given as Euler angles")
        .def("set_mode",        &franka_o80::State::set_mode,       "Sets state's value to mode value")
        .def("set_error",       &franka_o80::State::set_error,      "Sets state's value to error value")
        .def("get_type",        &franka_o80::State::get_type,       "Returns state's type")
        .def("get_real",        &franka_o80::State::get_real,       "Returns real value")
        .def("get_quaternion",  &franka_o80::State::get_quaternion, "Returns quaternion value")
        .def("get_wxyz",        &franka_o80::State::get_wxyz,       "Returns quaternion value given as wxyz")
        .def("get_euler",       &franka_o80::State::get_euler,      "Returns quaternion value given as Euler angles")
        .def("get_mode",        &franka_o80::State::get_mode,       "Returns mode value")
        .def("get_error",       &franka_o80::State::get_error,      "Returns error value")
        .def("to_string",       &franka_o80::State::to_string,      "Returns string representation of state")
        .def(pybind11::self == pybind11::self)
        .def(pybind11::self != pybind11::self),
        "Type",                                                     "Enumeration of types of state")
        .value("real",          franka_o80::State::Type::real,      "Real number")
        .value("quaternion",    franka_o80::State::Type::quaternion,"Quaternion")
        .value("mode",          franka_o80::State::Type::mode,      "franka_o80.Mode")
        .value("error",         franka_o80::State::Type::error,     "franka_o80.Error");
    
    //states.hpp
    m.def("default_states", &franka_o80::default_states,    "Returns robot's default states");

    //version.hpp
    m.def("version_major",  []() -> int { return franka_o80::version_major; }, "Major version");
    m.def("version_minor",  []() -> int { return franka_o80::version_minor; }, "Minor version");
    m.def("version_patch",  []() -> int { return franka_o80::version_patch; }, "Patch numebr");
}