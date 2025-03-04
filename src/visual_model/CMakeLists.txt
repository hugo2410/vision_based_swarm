# Define the source files for the simulation library
set(VISUAL_MODEL_SOURCES
        VisualModel2D.cpp
)

# Define the header files
set(VISUAL_MODEL_HEADERS
        VisualModel2D.h
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Create a library for simulation components
add_library(visual_model STATIC ${VISUAL_MODEL_SOURCES} ${VISUAL_MODEL_HEADERS})

# Specify include directories for the library
target_include_directories(visual_model PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}  # Current directory
        ${CMAKE_CURRENT_SOURCE_DIR}/..  # Parent directory (src)
        ${EIGEN3_INCLUDE_DIR}
)
# Add this to your CMakeLists.txt file
add_definitions(-DSOURCE_DIR="${CMAKE_SOURCE_DIR}")
