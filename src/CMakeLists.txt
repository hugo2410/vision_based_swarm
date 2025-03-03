# Add the simulation subdirectory
add_subdirectory(simulation)

# Add the main executable
add_executable(vision_based_swarm main.cpp)

# Link against the simulation library
target_link_libraries(vision_based_swarm PRIVATE simulation)

# Make sure the executable can find the simulation headers
target_include_directories(vision_based_swarm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
