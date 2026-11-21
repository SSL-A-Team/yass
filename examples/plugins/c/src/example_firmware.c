#include "yass_corelib.h"
#include <stdio.h>

void* yass_load_example_firmware(yass_TeamColor color, const char* config) {
    printf("Example firmware loaded.\n");
    return NULL;
}

void yass_run_example_firmware(yass_SimulationState state, uint64_t delta_t_us, void* user_data, yass_RobotCommand commands_out[16]) {
    // Populate commands_out
}

void yass_unload_example_firmware(void* user_data) {
    printf("Example firmware unloaded.\n");
}
