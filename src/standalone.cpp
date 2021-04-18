#include "../include/franka_o80/standalone.hpp"

franka_o80::Standalone::Standalone(std::shared_ptr<Driver> driver_ptr, double frequency, std::string segment_id)
    : o80::Standalone<queue_size, actuator_number, Driver, State, o80::VoidExtendedState>(driver_ptr, frequency, segment_id)
{
}

franka_o80::DriverIn franka_o80::Standalone::convert(const o80::States<actuator_number, State> &states)
{
    DriverIn result;
    for (size_t i = 0; i < 7; i++)
    {
        result.joint_positions.q[i] = states.get(i).value;
    }
	result.gripper_width = states.get(7).value;
    return result;
}

o80::States<franka_o80::actuator_number, franka_o80::State> franka_o80::Standalone::convert(const DriverOut &observation)
{
    o80::States<actuator_number, State> result;
	for (size_t i = 0; i < 7; i++)
    {
        result.set(i, observation.joint_positions.q[i]);
    }
	result.set(7, observation.gripper_width);
	return result;
}

// optional user function for adding content to extended_state based
// on observation. Not needed for this example.
/*
void Standalone::enrich_extended_state(o80::VoidExtendedState &extended_state,
                                       const DriverOut &observation)
{
}
*/

std::string franka_o80::get_segment_id(int id)
{
    return std::string(segment_id_base) + std::string("_") + std::to_string(id);
}