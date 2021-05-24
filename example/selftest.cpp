#include "../include/franka_o80/front_end.hpp"
#include "../include/franka_o80/standalone.hpp"
#include "../include/franka_o80/indexes.hpp"
#include "../include/franka_o80/constants.hpp"
#include "../include/franka_o80/errors.hpp"
#include <gtest/gtest.h>
#include <thread>

static const std::string segment_id("franka_o80_gtest");

TEST(RobotTest, DummyControl)
{
    //Start Standalone
    EXPECT_FALSE(o80::standalone_is_running(segment_id));
    EXPECT_NO_THROW((o80::start_standalone<franka_o80::Driver, franka_o80::Standalone, std::string>(segment_id, 1000.0, false, "192.168.0.1")));
    EXPECT_TRUE(o80::standalone_is_running(segment_id));

    //Start frontend
    std::unique_ptr<franka_o80::FrontEnd> frontend;
    EXPECT_NO_THROW(frontend = std::unique_ptr<franka_o80::FrontEnd>(new franka_o80::FrontEnd(segment_id)));

    //Check output
    o80::States<franka_o80::actuator_number, franka_o80::State> states = frontend->wait_for_next().get_observed_states();
    EXPECT_EQ(states.get(franka_o80::robot_positions[0]).get(), 0.0);
    EXPECT_EQ(states.get(franka_o80::robot_velocities[0]).get(), 0.0);
    EXPECT_EQ(states.get(franka_o80::control_positions).get(), 0.0);
    EXPECT_EQ(states.get(franka_o80::control_velocities).get(), 0.0);
    EXPECT_EQ(states.get(franka_o80::control_torques).get(), 0.0);
    EXPECT_EQ(states.get(franka_o80::control_error).get(), 0.0);
    EXPECT_EQ(states.get(franka_o80::control_reset).get(), 0.0);

    //Stop
    EXPECT_NO_THROW(frontend.release());
    EXPECT_NO_THROW(o80::stop_standalone(segment_id));
    EXPECT_FALSE(o80::standalone_is_running(segment_id));
}

TEST(RobotTest, PositionControl)
{
    //Start Standalone
    EXPECT_FALSE(o80::standalone_is_running(segment_id));
    EXPECT_NO_THROW((o80::start_standalone<franka_o80::Driver, franka_o80::Standalone, std::string>(segment_id, 1000.0, false, "192.168.0.1")));
    EXPECT_TRUE(o80::standalone_is_running(segment_id));

    //Start frontend
    std::unique_ptr<franka_o80::FrontEnd> frontend;
    EXPECT_NO_THROW(frontend = std::unique_ptr<franka_o80::FrontEnd>(new franka_o80::FrontEnd(segment_id)));

    //Set input
    EXPECT_NO_THROW(frontend->add_command(franka_o80::control_positions, franka_o80::State(1.0), o80::Duration_us::seconds(1), o80::Mode::QUEUE));
    
    //Wait
    EXPECT_NO_THROW(frontend->pulse_and_wait());
    for (size_t i = 0; i < 2000; i++) frontend->wait_for_next();

    //Check output
    o80::States<franka_o80::actuator_number, franka_o80::State> states = frontend->wait_for_next().get_observed_states();
    EXPECT_EQ(states.get(franka_o80::robot_positions[0]).get(), 0.0);
    EXPECT_EQ(states.get(franka_o80::robot_velocities[0]).get(), 0.0);
    EXPECT_EQ(states.get(franka_o80::control_positions).get(), 1.0);
    EXPECT_EQ(states.get(franka_o80::control_velocities).get(), 0.0);
    EXPECT_EQ(states.get(franka_o80::control_torques).get(), 0.0);
    EXPECT_EQ(states.get(franka_o80::control_error).get(), 0.0);
    EXPECT_EQ(states.get(franka_o80::control_reset).get(), 0.0);

    //Stop
    EXPECT_NO_THROW(frontend.release());
    EXPECT_NO_THROW(o80::stop_standalone(segment_id));
    EXPECT_FALSE(o80::standalone_is_running(segment_id));
}

TEST(RobotTest, BadPositionControl)
{
    //Start Standalone
    EXPECT_FALSE(o80::standalone_is_running(segment_id));
    EXPECT_NO_THROW((o80::start_standalone<franka_o80::Driver, franka_o80::Standalone, std::string>(segment_id, 1000.0, false, "192.168.0.1")));
    EXPECT_TRUE(o80::standalone_is_running(segment_id));

    //Start frontend
    std::unique_ptr<franka_o80::FrontEnd> frontend;
    EXPECT_NO_THROW(frontend = std::unique_ptr<franka_o80::FrontEnd>(new franka_o80::FrontEnd(segment_id)));

    //Set input
    EXPECT_NO_THROW(frontend->add_command(franka_o80::control_positions, franka_o80::State(1.0), o80::Duration_us::seconds(0), o80::Mode::QUEUE));
    
    //Wait
    EXPECT_NO_THROW(frontend->pulse_and_wait());
    for (size_t i = 0; i < 2000; i++) frontend->wait_for_next();

    //Check output
    o80::States<franka_o80::actuator_number, franka_o80::State> states = frontend->wait_for_next().get_observed_states();
    EXPECT_EQ(states.get(franka_o80::control_positions).get(), 1.0);
    EXPECT_EQ(states.get(franka_o80::control_velocities).get(), 0.0);
    EXPECT_EQ(states.get(franka_o80::control_torques).get(), 0.0);
    EXPECT_EQ(states.get(franka_o80::control_error).get(), 0.0);
    EXPECT_EQ(states.get(franka_o80::control_reset).get(), 0.0);

    //Set bad input
    EXPECT_NO_THROW(frontend->add_command(franka_o80::robot_positions[0], franka_o80::State(10.0), o80::Duration_us::seconds(1), o80::Mode::QUEUE));

    //Wait
    EXPECT_NO_THROW(frontend->pulse_and_wait());
    for (size_t i = 0; i < 2000; i++) frontend->wait_for_next();

    //Check bad output
    states = frontend->wait_for_next().get_observed_states();
    EXPECT_EQ(states.get(franka_o80::control_error).get(), franka_o80::error_robot_control_exception);
    
    //Stop
    EXPECT_NO_THROW(frontend.release());
    EXPECT_NO_THROW(o80::stop_standalone(segment_id));
    EXPECT_FALSE(o80::standalone_is_running(segment_id));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}