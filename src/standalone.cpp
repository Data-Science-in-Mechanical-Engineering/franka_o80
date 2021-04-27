#include "../include/franka_o80/standalone.hpp"

franka_o80::Standalone::Standalone(std::shared_ptr<Driver> driver_ptr, double frequency, std::string segment_id)
    : o80::Standalone<queue_size, actuator_number, Driver, State, o80::VoidExtendedState>(driver_ptr, frequency, segment_id)
{
}

franka_o80::DriverInput franka_o80::Standalone::convert(const o80::States<actuator_number, State> &states)
{
    return states;
}

o80::States<franka_o80::actuator_number, franka_o80::State> franka_o80::Standalone::convert(const DriverOutput &driver_output)
{
    return driver_output;
}

// optional user function for adding content to extended_state based
// on observation. Not needed for this example.
/*
void Standalone::enrich_extended_state(o80::VoidExtendedState &extended_state,
                                       const DriverOut &observation)
{
}
*/