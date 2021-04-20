#include <o80/pybind11_helper.hpp>
#include "../include/franka_o80/standalone.hpp"

PYBIND11_MODULE(franka_o80, m)
{
  o80::create_python_bindings<franka_o80::Standalone>(m);
  o80::create_standalone_python_bindings<franka_o80::Driver, franka_o80::Standalone, std::string>(m);
}
