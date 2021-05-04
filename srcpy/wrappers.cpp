#include <o80/pybind11_helper.hpp>
#include "../include/franka_o80/standalone.hpp"
#include "../include/franka_o80/segment_id.hpp"
#include "../include/franka_o80/constants.hpp"
#include "../include/franka_o80/errors.hpp"
#include "../include/franka_o80/indexes.hpp"
#include <stdexcept>

PYBIND11_MODULE(franka_o80, m)
{
    o80::create_python_bindings<franka_o80::Standalone>(m);
    o80::create_standalone_python_bindings<franka_o80::Driver, franka_o80::Standalone, std::string>(m);

    //segment_id.hpp
    m.def("get_segment_id", franka_o80::get_segment_id);

    //constants.hpp
    m.attr("robot_positions_min")       = franka_o80::robot_positions_min;
    m.attr("robot_positions_max")       = franka_o80::robot_positions_max;
    m.attr("robot_velocities_max")      = franka_o80::robot_velocities_max;
    m.attr("robot_accelerations_max")   = franka_o80::robot_accelerations_max;
    m.attr("robot_jerks_max")           = franka_o80::robot_jerks_max;
    m.attr("robot_torques_max")         = franka_o80::robot_torques_max;
    m.attr("robot_dtorques_max")        = franka_o80::robot_dtorques_max;
    m.attr("actuator_number")           = franka_o80::actuator_number;
    m.attr("queue_size")                = franka_o80::queue_size;
    m.attr("segment_id_base")           = franka_o80::segment_id_base;

    //errors.hpp
    m.attr("error_ok")                                  = franka_o80::error_ok;
    m.attr("error_robot_invalid_control")               = franka_o80::error_robot_invalid_control;
    m.attr("error_robot_command_exception")             = franka_o80::error_robot_command_exception;
    m.attr("error_robot_control_exception")             = franka_o80::error_robot_control_exception;
    m.attr("error_robot_invalid_operation_exception")   = franka_o80::error_robot_invalid_operation_exception;
    m.attr("error_robot_network_exception")             = franka_o80::error_robot_network_exception;
    m.attr("error_robot_realtime_exception")            = franka_o80::error_robot_realtime_exception;
    m.attr("error_robot_invalid_argument_exception")    = franka_o80::error_robot_invalid_argument_exception;
    m.attr("error_robot_other_exception")               = franka_o80::error_robot_other_exception;
    m.attr("error_gripper_command_exception")           = franka_o80::error_gripper_command_exception;
    m.attr("error_gripper_network_exception")           = franka_o80::error_gripper_network_exception;
    m.attr("error_gripper_invalid_operation_exception") = franka_o80::error_gripper_invalid_operation_exception;
    m.attr("error_gripper_other_exception")             = franka_o80::error_gripper_other_exception;

    //indexes.hpp
    m.def("robot_positions",        [](int i) -> int { if (i < 0 || i > 6) throw std::range_error("franka_o80 invaid joint index"); return franka_o80::robot_positions[i]; });
    m.def("robot_velocities",       [](int i) -> int { if (i < 0 || i > 6) throw std::range_error("franka_o80 invaid joint index"); return franka_o80::robot_velocities[i]; });
    m.def("robot_torques",          [](int i) -> int { if (i < 0 || i > 6) throw std::range_error("franka_o80 invaid joint index"); return franka_o80::robot_torques[i]; });
    m.attr("gripper_width")         = franka_o80::gripper_width;
    m.attr("gripper_temperature")   = franka_o80::gripper_temperature;
    m.attr("control_positions")     = franka_o80::control_positions;
    m.attr("control_velocities")    = franka_o80::control_velocities;
    m.attr("control_torques")       = franka_o80::control_torques;
    m.attr("control_error")         = franka_o80::control_error;
    m.attr("control_reset")         = franka_o80::control_reset;
}