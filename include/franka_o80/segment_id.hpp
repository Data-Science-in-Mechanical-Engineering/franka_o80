#pragma once

#include "constants.hpp"
#include <string>

namespace franka_o80
{
///Returns default segment id for given integer id
///@param id Integer identifier
std::string get_segment_id(int id);
} // namespace franka_o80