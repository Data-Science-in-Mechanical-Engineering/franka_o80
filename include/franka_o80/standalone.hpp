#pragma once

#include <o80/memory_clearing.hpp>
#include <o80/standalone.hpp>
#include "driver.hpp"
#include "state.hpp"
#include "constants.hpp"

namespace franka_o80
{
///Standalone, used to read and write to `Driver` at specified frequency
class Standalone : public o80::Standalone<queue_size, actuator_number, Driver, State, o80::VoidExtendedState>
{
public:
    ///Creates standalone
    ///@param driver_ptr `Driver` to be maintained
    ///@param frequency Frequency of reading and writing to `Driver`
    ///@param segment_id Identifier of shared memory
    Standalone(std::shared_ptr<Driver> driver_ptr, double frequency, std::string segment_id);
    ///Converts array of states to driver input
    ///@param states Array of states
    DriverInput convert(const o80::States<actuator_number, State> &states);
    ///Converts driver output to array of states
    ///@param driver_output Driver output
    o80::States<actuator_number, State> convert(const DriverOutput &driver_output);
};

///Returns default segment id for given integer id
///@param id Integer identifier
std::string get_segment_id(int id);
} // namespace franka_o80