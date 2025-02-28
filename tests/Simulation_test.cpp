//
// Copyright 2025 Hugo Birch
// created by hugo on 2/28/25.
//

#include "simulation/Simulation.h"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

class SimulationTest : public ::testing::Test {
 protected:
  // Default simulation parameters for tests
  SimulationParams getDefaultParams() {
    SimulationParams params;
    params.numAgents = 10;
    params.worldSizeX = 100.0;
    params.worldSizeY = 100.0;
    params.timeStep = 0.1;
    params.maxInitialVelocity = 2.0;
    params.maxInitialAcceleration = 1.0;
    params.initPositionType = "grid";
    return params;
  }
};

TEST_F(SimulationTest, DefaultInitialization) {
  SimulationParams params = getDefaultParams();
  Simulation simulation(params);

  // Check that agents vector is correctly sized
  EXPECT_EQ(simulation.getAgents().size(), params.numAgents);

  // Check that trajectories are correctly set up
  EXPECT_EQ(simulation.getTrajectories().size(), params.numAgents);

  // Check initial time
  EXPECT_DOUBLE_EQ(simulation.getCurrentTime(), 0.0);
}

TEST_F(SimulationTest, GridInitialization) {
  SimulationParams params = getDefaultParams();
  params.initPositionType = "grid";
  params.numAgents = 4;  // 2x2 grid
  params.worldSizeX = 10.0;
  params.worldSizeY = 10.0;

  Simulation simulation(params);
  simulation.initialize();

  // Expected positions in a 2x2 grid with 10x10 world:
  // (2.5, 2.5), (7.5, 2.5), (2.5, 7.5), (7.5, 7.5)
  auto& agents = simulation.getAgents();

  // Check if agents are roughly in grid positions
  // We can't check exact positions due to randomness in velocity/acceleration
  EXPECT_NEAR(agents[0].getPosition().first, 2.5, 0.01);
  EXPECT_NEAR(agents[0].getPosition().second, 2.5, 0.01);

  EXPECT_NEAR(agents[1].getPosition().first, 7.5, 0.01);
  EXPECT_NEAR(agents[1].getPosition().second, 2.5, 0.01);

  EXPECT_NEAR(agents[2].getPosition().first, 2.5, 0.01);
  EXPECT_NEAR(agents[2].getPosition().second, 7.5, 0.01);

  EXPECT_NEAR(agents[3].getPosition().first, 7.5, 0.01);
  EXPECT_NEAR(agents[3].getPosition().second, 7.5, 0.01);

  // Check that all velocities are within bounds
  for (const auto& agent : agents) {
    EXPECT_LE(std::abs(agent.getVelocity().first), params.maxInitialVelocity);
    EXPECT_LE(std::abs(agent.getVelocity().second), params.maxInitialVelocity);
  }

  // Check that all accelerations are within bounds
  for (const auto& agent : agents) {
    EXPECT_LE(std::abs(agent.getAcceleration().first),
              params.maxInitialAcceleration);
    EXPECT_LE(std::abs(agent.getAcceleration().second),
              params.maxInitialAcceleration);
  }
}

TEST_F(SimulationTest, RandomInitialization) {
  SimulationParams params = getDefaultParams();
  params.initPositionType = "random";

  Simulation simulation(params);
  simulation.initialize();

  // Check that agents are initialized within world bounds
  for (const auto& agent : simulation.getAgents()) {
    EXPECT_GE(agent.getPosition().first, 0.0);
    EXPECT_LE(agent.getPosition().first, params.worldSizeX);

    EXPECT_GE(agent.getPosition().second, 0.0);
    EXPECT_LE(agent.getPosition().second, params.worldSizeY);

    // Check velocity and acceleration bounds
    EXPECT_LE(std::abs(agent.getVelocity().first), params.maxInitialVelocity);
    EXPECT_LE(std::abs(agent.getVelocity().second), params.maxInitialVelocity);

    EXPECT_LE(std::abs(agent.getAcceleration().first),
              params.maxInitialAcceleration);
    EXPECT_LE(std::abs(agent.getAcceleration().second),
              params.maxInitialAcceleration);
  }
}

TEST_F(SimulationTest, UpdateAgents) {
  SimulationParams params = getDefaultParams();
  params.numAgents = 1;  // Use just one agent for simpler testing
  params.timeStep = 1.0;

  Simulation simulation(params);
  simulation.initialize();

  // Set known position, velocity, and acceleration
  simulation.getAgents()[0].setPosition({0.0, 0.0});
  simulation.getAgents()[0].setVelocity({1.0, 2.0});
  simulation.getAgents()[0].setAcceleration({0.5, -0.5});

  // Get initial trajectory size for reference
  size_t initialTrajectorySize = simulation.getTrajectories()[0].size();

  // Update simulation for one time step
  simulation.update();

  // Check the actual position after update
  EXPECT_DOUBLE_EQ(simulation.getAgents()[0].getPosition().first, 1.5);
  EXPECT_DOUBLE_EQ(simulation.getAgents()[0].getPosition().second, 1.5);

  // Check that current time was updated
  EXPECT_DOUBLE_EQ(simulation.getCurrentTime(), params.timeStep);

  // Check that trajectory was recorded (one more entry than before)
  const auto& trajectories = simulation.getTrajectories();
  EXPECT_EQ(trajectories[0].size(), initialTrajectorySize + 1);

  // Check the latest trajectory entry matches the current position
  EXPECT_DOUBLE_EQ(trajectories[0].back().first, 1.5);
  EXPECT_DOUBLE_EQ(trajectories[0].back().second, 1.5);
}

TEST_F(SimulationTest, MultipleUpdates) {
  SimulationParams params = getDefaultParams();
  params.numAgents = 1;
  params.timeStep = 0.5;

  Simulation simulation(params);
  simulation.initialize();

  // Set known position, velocity, and acceleration
  simulation.getAgents()[0].setPosition({0.0, 0.0});
  simulation.getAgents()[0].setVelocity({1.0, 1.0});
  simulation.getAgents()[0].setAcceleration({0.0, 0.0});

  // Get initial trajectory size for reference
  size_t initialTrajectorySize = simulation.getTrajectories()[0].size();

  // Update multiple times
  for (int i = 0; i < 4; i++) {
    simulation.update();
  }

  // Position after 4 updates of 0.5s with velocity (1,1) and no acceleration
  EXPECT_DOUBLE_EQ(simulation.getAgents()[0].getPosition().first, 2.0);
  EXPECT_DOUBLE_EQ(simulation.getAgents()[0].getPosition().second, 2.0);

  // Check current time: 4 updates * 0.5s
  EXPECT_DOUBLE_EQ(simulation.getCurrentTime(), 2.0);

  // Check trajectory size: initial + 4 updates
  EXPECT_EQ(simulation.getTrajectories()[0].size(), initialTrajectorySize + 4);
}

TEST_F(SimulationTest, SetAgentAccelerations) {
  SimulationParams params = getDefaultParams();
  params.numAgents = 3;

  Simulation simulation(params);
  simulation.initialize();

  // Set new accelerations
  std::vector<std::pair<double, double>> accels = {
      {1.0, 2.0}, {-0.5, 0.5}, {0.0, -1.0}};

  simulation.setAgentAccelerations(accels);

  // Check that accelerations were set correctly
  EXPECT_DOUBLE_EQ(simulation.getAgents()[0].getAcceleration().first, 1.0);
  EXPECT_DOUBLE_EQ(simulation.getAgents()[0].getAcceleration().second, 2.0);

  EXPECT_DOUBLE_EQ(simulation.getAgents()[1].getAcceleration().first, -0.5);
  EXPECT_DOUBLE_EQ(simulation.getAgents()[1].getAcceleration().second, 0.5);

  EXPECT_DOUBLE_EQ(simulation.getAgents()[2].getAcceleration().first, 0.0);
  EXPECT_DOUBLE_EQ(simulation.getAgents()[2].getAcceleration().second, -1.0);
}

TEST_F(SimulationTest, SetAgentAccelerationsWithMismatchedSizes) {
  SimulationParams params = getDefaultParams();
  params.numAgents = 3;

  Simulation simulation(params);
  simulation.initialize();

  // Reset accelerations to known values
  for (auto& agent : simulation.getAgents()) {
    agent.setAcceleration({0.0, 0.0});
  }

  // Set new accelerations with smaller vector
  std::vector<std::pair<double, double>> accels = {{1.0, 2.0}, {-0.5, 0.5}};

  // This should handle the size mismatch gracefully with a warning
  simulation.setAgentAccelerations(accels);

  // Check that accelerations were set correctly for the provided agents
  EXPECT_DOUBLE_EQ(simulation.getAgents()[0].getAcceleration().first, 1.0);
  EXPECT_DOUBLE_EQ(simulation.getAgents()[0].getAcceleration().second, 2.0);

  EXPECT_DOUBLE_EQ(simulation.getAgents()[1].getAcceleration().first, -0.5);
  EXPECT_DOUBLE_EQ(simulation.getAgents()[1].getAcceleration().second, 0.5);

  // Third agent should be unchanged
  EXPECT_DOUBLE_EQ(simulation.getAgents()[2].getAcceleration().first, 0.0);
  EXPECT_DOUBLE_EQ(simulation.getAgents()[2].getAcceleration().second, 0.0);
}

TEST_F(SimulationTest, InvalidInitializationType) {
  SimulationParams params = getDefaultParams();
  params.initPositionType = "invalid_type";

  // Testing with an invalid initialization type should default to grid
  Simulation simulation(params);

  // Capture stderr to verify warning message
  testing::internal::CaptureStderr();

  simulation.initialize();

  std::string output = testing::internal::GetCapturedStderr();
  EXPECT_TRUE(output.find("Unknown initialization type") != std::string::npos);
  EXPECT_TRUE(output.find("Defaulting to grid initialization") !=
              std::string::npos);

  // Verify agents were initialized using grid method
  // For a 10-agent simulation in a 100x100 world, grid size would be 4x4
  // Check first agent is properly positioned (not at origin)
  auto& agents = simulation.getAgents();
  EXPECT_GT(agents[0].getPosition().first, 0.0);
  EXPECT_GT(agents[0].getPosition().second, 0.0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
