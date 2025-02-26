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

# Add src directory
add_subdirectory(src)

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
