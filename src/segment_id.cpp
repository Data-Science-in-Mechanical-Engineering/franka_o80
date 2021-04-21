#include "../include/franka_o80/segment_id.hpp"

std::string franka_o80::get_segment_id(int id)
{
    return std::string(segment_id_base) + std::string("_") + std::to_string(id);
}