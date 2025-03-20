cmake_minimum_required(VERSION 3.14)
project(vision_based_swarm VERSION 0.1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable warnings
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Define source directory for scripts
add_definitions(-DSOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# Enable testing
enable_testing()

# Add code coverage option
option(CODE_COVERAGE "Enable coverage reporting" OFF)
if(CODE_COVERAGE AND NOT MSVC)
    # Add required flags for coverage (gcc/clang only)
    add_compile_options(-O0 -g --coverage)
    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.13)
        add_link_options(--coverage)
    else()
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
    endif()
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Create the main library target (excluding main.cpp)
file(GLOB_RECURSE PROJECT_SOURCES "src/*.cpp")
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*main\\.cpp$")
add_library(${PROJECT_NAME}_lib ${PROJECT_SOURCES}
        tests/Simulation_test.cpp)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIR}
)

# Add src directory (for the executable)
add_subdirectory(src)

# Add tests directory if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    add_subdirectory(tests)
endif()

# Add option for building documentation
option(BUILD_DOCUMENTATION "Build Doxygen documentation" OFF)

# Configure Doxygen if requested and available
if(BUILD_DOCUMENTATION)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        # Set Doxygen configuration variables
        set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/docs)
        set(DOXYGEN_PROJECT_NAME "Vision Based Swarm")
        set(DOXYGEN_PROJECT_BRIEF "A simulation framework for vision-based swarm robotics")
        set(DOXYGEN_EXTRACT_ALL YES)
        set(DOXYGEN_EXTRACT_PRIVATE YES)
        set(DOXYGEN_EXTRACT_STATIC YES)

        # Configure Doxyfile
        doxygen_add_docs(
                docs
                ${CMAKE_CURRENT_SOURCE_DIR}/src
                COMMENT "Generating documentation with Doxygen"
        )
    else()
        message(STATUS "Doxygen not found, documentation will not be built")
    endif()
endif()
