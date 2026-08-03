#include "yass-corelib.hpp"
#include <iostream>

extern "C" {

void run_firmware_frame(yass::SimulationState state, yass::TeamColor color, uint64_t delta_t_us, yass::RobotCommand commands_out[16])
{
    std::cout << "run_firmware_frame from example C++ plugins\n";
    commands_out[0].motion_command.tag = yass::MotionCommand::Tag::Velocity;
    commands_out[0].motion_command.velocity.x = 0.1;
}

void run_vision_frame(yass::SimulationState state)
{
    std::cout << "run_vision_frame from example C++ plugins\n";
}

}
