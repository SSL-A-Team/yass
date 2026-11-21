#include "yass_corelib.hpp"
#include <iostream>

extern "C" {

void* yass_load_example_vision(yass::FieldDimensions field_dimensions, const char* config) {
    printf("Example vision loaded.\n");
    return nullptr;
}

void yass_run_example_vision(yass::SimulationState state, void* user_data) {
    // Populate commands_out
}

void yass_unload_example_vision(void* user_data) {
    printf("Example vision unloaded.\n");
}

}
