# Find GTest package
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Add test executables
file(GLOB TEST_SOURCES "*.cpp")
add_executable(unit_tests ${TEST_SOURCES})

# Link against project library and GTest
target_link_libraries(unit_tests
        ${PROJECT_NAME}_lib
        GTest::GTest
        GTest::Main
)

# Add to test suite
add_test(NAME unit_tests COMMAND unit_tests)