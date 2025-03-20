# Add the simulation subdirectory
add_subdirectory(simulation)

# Add the simulation subdirectory
add_subdirectory(visual_model)

# Add the main executable
add_executable(vision_based_swarm main.cpp)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Link against the simulation library
target_link_libraries(vision_based_swarm PRIVATE simulation visual_model)

# Make sure the executable can find the simulation headers
target_include_directories(vision_based_swarm PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/simulation
        ${CMAKE_CURRENT_SOURCE_DIR}/visual_model
        ${EIGEN3_INCLUDE_DIR}
)
