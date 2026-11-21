#include "yass_corelib.h"
#include <stdio.h>

void* yass_load_example_vision(yass_FieldDimensions field_dimensions, const char* config) {
    printf("Example vision loaded.\n");
    return NULL;
}

void yass_run_example_vision(yass_SimulationState state, void* user_data) {
    // Populate commands_out
}

void yass_unload_example_vision(void* user_data) {
    printf("Example vision unloaded.\n");
}
