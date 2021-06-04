#include "../include/franka_o80/states.hpp"

franka_o80::States franka_o80::default_states()
{
    States states;
    for (size_t i = 0; i < actuator_number; i++) states.values[i] = 0.0;
    states.values[franka_o80::robot_positions[0]] = 0.0;
    states.values[franka_o80::robot_positions[1]] = -M_PI / 4;
    states.values[franka_o80::robot_positions[2]] = 0.0;
    states.values[franka_o80::robot_positions[3]] = -3 * M_PI / 4;
    states.values[franka_o80::robot_positions[4]] = 0.0;
    states.values[franka_o80::robot_positions[5]] = M_PI / 2;
    states.values[franka_o80::robot_positions[6]] = M_PI / 4;
    return states;
};