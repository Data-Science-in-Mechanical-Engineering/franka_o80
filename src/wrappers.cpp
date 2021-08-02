#include <o80/pybind11_helper.hpp>
#include "../include/franka_o80/kinematics.hpp"
#include "../include/franka_o80/standalone.hpp"
#include "../include/franka_o80/limits.hpp"
#include "../include/franka_o80/error.hpp"
#include "../include/franka_o80/actuator.hpp"
#include <stdexcept>

PYBIND11_MODULE(franka_o80, m)
{
    o80::create_python_bindings<franka_o80::Standalone, o80::NO_STATE>(m);
    o80::create_standalone_python_bindings<franka_o80::Driver, franka_o80::Standalone, std::string>(m);
    //state.hpp
    pybind11::class_<franka_o80::State>(m, "State")
        .def(pybind11::init<>())
        .def(pybind11::init<double>())
        .def(pybind11::init<const Eigen::Quaterniond&>())
        .def(pybind11::init<const Eigen::Matrix<double, 4, 1>&>())
        .def(pybind11::init<const Eigen::Matrix<double, 3, 1>&>())
        .def(pybind11::init<franka_o80::Mode>())
        .def(pybind11::init<franka_o80::Error>())
        .def("get_real",        &franka_o80::State::get_real)
        .def("get_quaternion",  &franka_o80::State::get_quaternion)
        .def("get_wxyz",        &franka_o80::State::get_wxyz)
        .def("get_euler",       &franka_o80::State::get_euler)
        .def("get_mode",        &franka_o80::State::get_mode)
        .def("get_error",       &franka_o80::State::get_error)
        .def("set_real",        &franka_o80::State::set_real)
        .def("set_quaternion",  &franka_o80::State::set_quaternion)
        .def("set_wxyz",        &franka_o80::State::set_wxyz)
        .def("set_euler",       &franka_o80::State::set_euler)
        .def("set_mode",        &franka_o80::State::set_mode)
        .def("set_error",       &franka_o80::State::set_error)
        .def("to_string", &franka_o80::State::to_string);

    //limits.hpp
    m.attr("joint_position_min")        = franka_o80::joint_position_min;
    m.attr("joint_position_max")        = franka_o80::joint_position_max;
    m.attr("joint_velocity_max")        = franka_o80::joint_velocity_max;
    m.attr("joint_acceleration_max")    = franka_o80::joint_acceleration_max;
    m.attr("joint_jerk_max")            = franka_o80::joint_jerk_max;
    m.attr("joint_torque_max")          = franka_o80::joint_torque_max;
    m.attr("joint_dtorque_max")         = franka_o80::joint_dtorque_max;
    m.attr("actuator_number")           = franka_o80::actuator_number;
    m.attr("queue_size")                = franka_o80::queue_size;

    //mode.hpp
    pybind11::enum_<franka_o80::Mode>(m, "Mode")
    .value("invalid", franka_o80::Mode::invalid)
    .value("torque", franka_o80::Mode::torque)
    .value("torque_position", franka_o80::Mode::torque_position)
	.value("torque_velocity", franka_o80::Mode::torque_velocity)
	.value("torque_cartesian_position", franka_o80::Mode::torque_cartesian_position)
	.value("torque_cartesian_velocity", franka_o80::Mode::torque_cartesian_velocity)
	.value("position", franka_o80::Mode::position)
	.value("velocity", franka_o80::Mode::velocity)
	.value("cartesian_position", franka_o80::Mode::cartesian_position)
	.value("cartesian_velocity", franka_o80::Mode::cartesian_velocity)
	.value("intelligent_position", franka_o80::Mode::intelligent_position)
	.value("intelligent_cartesian_position", franka_o80::Mode::intelligent_cartesian_position)
    .export_values();

    //error.hpp
    pybind11::enum_<franka_o80::Error>(m, "Error")
    .value("ok", franka_o80::Error::ok)
    .value("robot_command_exception", franka_o80::Error::robot_command_exception)
    .value("robot_control_exception", franka_o80::Error::robot_control_exception)
    .value("robot_invalid_operation_exception", franka_o80::Error::robot_invalid_operation_exception)
    .value("robot_network_exception", franka_o80::Error::robot_network_exception)
    .value("robot_realtime_exception", franka_o80::Error::robot_realtime_exception)
    .value("robot_invalid_argument_exception", franka_o80::Error::robot_invalid_argument_exception)
    .value("robot_other_exception", franka_o80::Error::robot_other_exception)
    .value("gripper_command_exception", franka_o80::Error::gripper_command_exception)
    .value("gripper_network_exception", franka_o80::Error::gripper_network_exception)
    .value("gripper_invalid_operation_exception", franka_o80::Error::gripper_invalid_operation_exception)
    .value("gripper_other_exception", franka_o80::Error::gripper_other_exception)
    .export_values();

    //actuator.hpp
    m.attr("control_mode")          = franka_o80::control_mode;
    m.attr("control_error")         = franka_o80::control_error;
    m.attr("control_reset")         = franka_o80::control_reset;
    m.attr("gripper_width")         = franka_o80::gripper_width;
    m.attr("gripper_temperature")   = franka_o80::gripper_temperature;
    m.def("joint_position",         [](int i) -> int { if (i < 0 || i > 6) throw std::range_error("franka_o80 invaid joint index"); return franka_o80::joint_position[i]; });
    m.def("joint_velocity",         [](int i) -> int { if (i < 0 || i > 6) throw std::range_error("franka_o80 invaid joint index"); return franka_o80::joint_velocity[i]; });
    m.def("joint_torque",           [](int i) -> int { if (i < 0 || i > 6) throw std::range_error("franka_o80 invaid joint index"); return franka_o80::joint_torque[i]; });
    m.def("cartesian_position",     [](int i) -> int { if (i < 0 || i > 3) throw std::range_error("franka_o80 invaid dimension index"); return franka_o80::cartesian_position[i]; });
    m.attr("cartesian_orientation") = franka_o80::cartesian_orientation;
    m.def("cartesian_velocity",     [](int i) -> int { if (i < 0 || i > 3) throw std::range_error("franka_o80 invaid dimension index"); return franka_o80::cartesian_velocity[i]; });
    m.def("cartesian_rotation",     [](int i) -> int { if (i < 0 || i > 3) throw std::range_error("franka_o80 invaid dimension index"); return franka_o80::cartesian_rotation[i]; });
    m.def("joint_stiffness",        [](int i) -> int { if (i < 0 || i > 7) throw std::range_error("franka_o80 invaid dimension index"); return franka_o80::joint_stiffness[i]; });
    m.def("joint_damping",          [](int i) -> int { if (i < 0 || i > 7) throw std::range_error("franka_o80 invaid dimension index"); return franka_o80::joint_damping[i]; });
    m.def("cartesian_stiffness",    [](int i) -> int { if (i < 0 || i > 6) throw std::range_error("franka_o80 invaid dimension index"); return franka_o80::cartesian_stiffness[i]; });
    m.def("cartesian_damping",      [](int i) -> int { if (i < 0 || i > 6) throw std::range_error("franka_o80 invaid dimension index"); return franka_o80::cartesian_damping[i]; });

    //kinematics.hpp
    m.def("joint_to_cartesian", &franka_o80::joint_to_cartesian);
    m.def("cartesian_to_joint", pybind11::overload_cast<franka_o80::States&>(&franka_o80::cartesian_to_joint));
    m.def("cartesian_to_joint", pybind11::overload_cast<franka_o80::States&, const franka_o80::States&>(&franka_o80::cartesian_to_joint));
}