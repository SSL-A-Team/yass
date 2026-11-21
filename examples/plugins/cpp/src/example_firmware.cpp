#include "yass_corelib.hpp"
#include <iostream>

extern "C" {

void* yass_load_example_firmware(yass::TeamColor color, const char* config) {
    std::cout << "Example firmware loaded." << std::endl;
    return nullptr;
}

void yass_run_example_firmware(yass::SimulationState state, uint64_t delta_t_us, void* user_data, yass::RobotCommand commands_out[16]) {
    // Populate commands_out
}

void yass_unload_example_firmware(void* user_data) {
    std::cout << "Example firmware unloaded." << std::endl;
}

}
