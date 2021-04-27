#include "../include/franka_o80/driver_input_output.hpp"

franka_o80::DriverInputOutput::DriverInputOutput()
{}

franka_o80::DriverInputOutput::DriverInputOutput(const o80::States<actuator_number, State> &states) : o80::States<actuator_number, State>(states)
{}

std::string franka_o80::DriverInputOutput::to_string(bool output) const
{
    //Status
    std::string result;
    result += " [ ";
    if (!output) { result += " p: "; result += std::to_string(get(control_positions).get()); }
    if (!output) { result += " v: "; result += std::to_string(get(control_velocities).get()); }
    if (!output) { result += " t: "; result += std::to_string(get(control_torques).get()); }
    if (output) { result += " e: "; result += std::to_string(get(control_error).get()); }
    if (!output) { result += " r: "; result += std::to_string(get(control_reset).get()); }
    result += " ] ";

    //Robot
    for (size_t i = 0; i < 7; i++)
    {
        result += " [ ";
        if (get(control_positions) == 1.0) { result += " p: "; result += get(robot_positions[i]).get(); }
        if (get(control_velocities) == 1.0) { result += " v: "; result += get(robot_velocities[i]).get(); }
        if (get(control_torques) == 1.0) { result += " t: "; result += get(robot_torques[i]).get(); }
        result += " ] ";
    }
    //Gripper
    result += "+";
    result += " [ ";
    result += " w: "; result += std::to_string(get(gripper_width).get());
    if (output) { result += " t: "; result += get(gripper_temperature).get(); }
    result += " ] ";

    return result;
}