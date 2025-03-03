#!/bin/bash
set -e

# Clean previous coverage data
lcov --directory . --zerocounters

# Configure CMake with coverage enabled
cmake -S . -B build -DCODE_COVERAGE=ON
cmake --build build

# Run tests
cd build
ctest --verbose

# Collect coverage data
lcov --directory . --capture --output-file coverage.info
lcov --remove coverage.info '/usr/*' '*/tests/*' '*/build/*' --output-file coverage.info
lcov --list coverage.info

# Generate HTML report
genhtml coverage.info --output-directory coverage_report
