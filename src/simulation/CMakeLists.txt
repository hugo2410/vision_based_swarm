# Create a library target for the simulation module.
add_library(simulation
        Simulation.cpp
        Agent.cpp)

# Include directories for headers.
target_include_directories(simulation PUBLIC ../../include)

# Link any required libraries (e.g., math libraries).
target_link_libraries(simulation PRIVATE m)
