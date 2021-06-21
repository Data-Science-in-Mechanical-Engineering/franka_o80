#include "../include/franka_o80/states.hpp"

franka_o80::States franka_o80::default_states()
{
    States states;
    for (size_t i = 0; i < actuator_number; i++) states.values[i] = 0.0;
    states.values[franka_o80::joint_position[0]] = 0.0;
    states.values[franka_o80::joint_position[1]] = -M_PI / 4;
    states.values[franka_o80::joint_position[2]] = 0.0;
    states.values[franka_o80::joint_position[3]] = -3 * M_PI / 4;
    states.values[franka_o80::joint_position[4]] = 0.0;
    states.values[franka_o80::joint_position[5]] = M_PI / 2;
    states.values[franka_o80::joint_position[6]] = M_PI / 4;
    return states;
};