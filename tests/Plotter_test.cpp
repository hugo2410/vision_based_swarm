// Copyright 2025 Hugo Birch
#include "simulation/Plotter.h"

#include <gtest/gtest.h>
#include <string.h>
#include <sys/stat.h>

#include <cstdio>  // For std::remove
#include <fstream>
#include <iostream>
#include <string>

#include "simulation/Simulation.h"
#include "simulation/SimulationParams.h"

/**
 * @brief Helper function to check if a file exists.
 */
bool fileExists(const std::string& filename) {
  struct stat buffer;
  return (stat(filename.c_str(), &buffer) == 0);
}

/**
 * @brief Helper function to read the contents of a file into a string.
 */
std::string readFile(const std::string& filename) {
  std::ifstream file(filename);
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

/**
 * @brief Test fixture for Plotter using an actual simulation.
 */
class PlotterTest : public ::testing::Test {
 protected:
  SimulationParams params;
  Simulation* simulation;
  Plotter* plotter;

  void SetUp() override {
    // Define simulation parameters
    params.numAgents = 3;
    params.worldSizeX = 100.0;
    params.worldSizeY = 100.0;
    params.timeStep = 0.1;
    params.maxInitialVelocity = 2.0;
    params.maxInitialAcceleration = 1.0;
    params.initPositionType =
        "random";  // Let the simulation place agents randomly

    // Run the actual simulation
    simulation = new Simulation(params);
    simulation->initialize();

    // Step the simulation forward to generate trajectory data
    for (int i = 0; i < 10; i++) {  // Simulate 10 time steps
      simulation->update();
    }

    // Create Plotter instance
    plotter = new Plotter(*simulation, params);
  }

  void TearDown() override {
    delete simulation;
    delete plotter;
    // Clean up generated files
    std::remove("trajectory_data.json");
  }
};

/**
 * @brief Test trajectory data file generation
 */
TEST_F(PlotterTest, GeneratesTrajectoryJSON) {
  std::string testFilename = "test_plot.png";

  // Call plot function (it should generate "trajectory_data.json")
  plotter->plotTrajectories(testFilename);

  // Check if JSON file was created
  ASSERT_TRUE(fileExists("trajectory_data.json"))
      << "Error: trajectory_data.json was not created!";

  // Read the file and print it for debugging
  std::string jsonData = readFile("trajectory_data.json");
  std::cout << "Generated JSON:\n" << jsonData << std::endl;

  // Ensure JSON structure is valid
  EXPECT_NE(jsonData.find("["), std::string::npos);
  EXPECT_NE(jsonData.find("]"), std::string::npos);
}

/**
 * @brief Test that the correct system command is generated for plotting
 */
TEST_F(PlotterTest, GeneratesPlotCommand) {
  std::string testFilename = "test_plot.png";

  // Redirect std::cout to capture output
  std::stringstream buffer;
  std::streambuf* old = std::cout.rdbuf(buffer.rdbuf());

  // Run plot function
  plotter->plotTrajectories(testFilename);

  // Restore std::cout
  std::cout.rdbuf(old);

  // Verify command output contains expected execution command
  std::string capturedOutput = buffer.str();
  EXPECT_NE(capturedOutput.find("python3"), std::string::npos);
  EXPECT_NE(capturedOutput.find("plot_trajectories.py"), std::string::npos);
  EXPECT_NE(capturedOutput.find("--data_file=trajectory_data.json"),
            std::string::npos);
  EXPECT_NE(capturedOutput.find("--output=test_plot.png"), std::string::npos);
}

/**
 * @brief Test video generation command
 */
TEST_F(PlotterTest, GeneratesVideoCommand) {
  std::string testFilename = "test_video.mp4";
  int fps = 30;

  // Redirect std::cout to capture output
  std::stringstream buffer;
  std::streambuf* old = std::cout.rdbuf(buffer.rdbuf());

  // Run createVideo function
  plotter->createVideo(testFilename, fps);

  // Restore std::cout
  std::cout.rdbuf(old);

  // Verify command output contains expected execution command
  std::string capturedOutput = buffer.str();
  EXPECT_NE(capturedOutput.find("python3"), std::string::npos);
  EXPECT_NE(capturedOutput.find("create_video.py"), std::string::npos);
  EXPECT_NE(capturedOutput.find("--data_file=trajectory_data.json"),
            std::string::npos);
  EXPECT_NE(capturedOutput.find("--output=test_video.mp4"), std::string::npos);
  EXPECT_NE(capturedOutput.find("--fps=30"), std::string::npos);
}
