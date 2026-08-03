#include "yass-corelib.h"
#include <stdio.h>

void run_firmware_frame(yass_SimulationState state, yass_TeamColor color, uint64_t delta_t_us, yass_RobotCommand commands_out[16])
{
    printf("run_firmware_frame from example C plugins\n");
    commands_out[0].motion_command.tag = yass_MotionCommand_Velocity;
    commands_out[0].motion_command.velocity.x = 0.1;
}

void run_vision_frame(yass_SimulationState state)
{
    printf("run_vision_frame from example C plugins\n");
}
