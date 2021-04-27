#include "../include/franka_o80/segment_id.hpp"

std::string franka_o80::get_segment_id(int id)
{
    return segment_id_base + std::to_string(id);
}