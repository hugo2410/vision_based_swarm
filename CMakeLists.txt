cmake_minimum_required(VERSION 3.10)
project(vision_based_swarm)

# Set C++ standard.
set(CMAKE_CXX_STANDARD 17)

# Add subdirectories.
add_subdirectory(src/Simulation)

# Add executable target for the main application.
add_executable(main src/main.cpp)

# Link the simulation and vision libraries to the main executable.
target_link_libraries(main simulation)
