# Define the source files for the simulation library
set(SIMULATION_SOURCES
        Agent.cpp
        Simulation.cpp
        Plotter.cpp
)

# Define the header files
set(SIMULATION_HEADERS
        Agent.h
        Simulation.h
        SimulationParams.h
        Plotter.h
        WaypointFollowing.h
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Create a library for simulation components
add_library(simulation STATIC ${SIMULATION_SOURCES} ${SIMULATION_HEADERS})

# Specify include directories for the library
target_include_directories(simulation PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}  # Current directory
        ${CMAKE_CURRENT_SOURCE_DIR}/..  # Parent directory (src)
        ${EIGEN3_INCLUDE_DIR}
)
# Add this to your CMakeLists.txt file
add_definitions(-DSOURCE_DIR="${CMAKE_SOURCE_DIR}")
