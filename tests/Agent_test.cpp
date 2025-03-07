// Copyright 2025 Hugo Birch

#include "simulation/Agent.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>

TEST(AgentTest, DefaultInitialization) {
  // Test default initialization of an agent
  Agent agent;

  auto position = agent.getPosition();
  auto velocity = agent.getVelocity();
  auto acceleration = agent.getAcceleration();

  // Default constructor should initialize to origin with zero
  // velocity/acceleration
  EXPECT_DOUBLE_EQ(0.0, position.x());
  EXPECT_DOUBLE_EQ(0.0, position.y());
  EXPECT_DOUBLE_EQ(0.0, velocity.x());
  EXPECT_DOUBLE_EQ(0.0, velocity.y());
  EXPECT_DOUBLE_EQ(0.0, acceleration.x());
  EXPECT_DOUBLE_EQ(0.0, acceleration.y());
}

TEST(AgentTest, SetPosition) {
  // Test setting position
  Agent agent;
  agent.setPosition(Eigen::Vector2d(10.0, 20.0));

  auto position = agent.getPosition();
  EXPECT_DOUBLE_EQ(10.0, position.x());
  EXPECT_DOUBLE_EQ(20.0, position.y());
}

TEST(AgentTest, SetVelocity) {
  // Test setting velocity
  Agent agent;
  agent.setVelocity(Eigen::Vector2d(5.0, -3.0));

  auto velocity = agent.getVelocity();
  EXPECT_DOUBLE_EQ(5.0, velocity.x());
  EXPECT_DOUBLE_EQ(-3.0, velocity.y());
}

TEST(AgentTest, SetAcceleration) {
  // Test setting acceleration
  Agent agent;
  agent.setAcceleration(Eigen::Vector2d(2.0, 4.0));

  auto acceleration = agent.getAcceleration();
  EXPECT_DOUBLE_EQ(2.0, acceleration.x());
  EXPECT_DOUBLE_EQ(4.0, acceleration.y());
}

TEST(AgentTest, UpdateStateWithZeroAcceleration) {
  // Test updating state with zero acceleration
  Agent agent;
  agent.setPosition(Eigen::Vector2d(1.0, 1.0));
  agent.setVelocity(Eigen::Vector2d(2.0, 3.0));
  agent.setAcceleration(Eigen::Vector2d(0.0, 0.0));

  agent.updateState(0.5);  // Update with timestep = 0.5

  // Position should change based on velocity
  auto position = agent.getPosition();
  EXPECT_DOUBLE_EQ(2.0, position.x());  // 1.0 + 2.0 * 0.5
  EXPECT_DOUBLE_EQ(2.5, position.y());  // 1.0 + 3.0 * 0.5

  // Velocity should remain unchanged with zero acceleration
  auto velocity = agent.getVelocity();
  EXPECT_DOUBLE_EQ(2.0, velocity.x());
  EXPECT_DOUBLE_EQ(3.0, velocity.y());
}

TEST(AgentTest, UpdateStateWithAcceleration) {
  // Test updating state with non-zero acceleration
  Agent agent;
  agent.setPosition(Eigen::Vector2d(0.0, 0.0));
  agent.setVelocity(Eigen::Vector2d(0.0, 0.0));
  agent.setAcceleration(Eigen::Vector2d(2.0, 4.0));

  agent.updateState(1.0);  // Update with timestep = 1.0

  // Velocity should change based on acceleration
  auto velocity = agent.getVelocity();
  EXPECT_DOUBLE_EQ(2.0, velocity.x());  // 0.0 + 2.0 * 1.0
  EXPECT_DOUBLE_EQ(4.0, velocity.y());  // 0.0 + 4.0 * 1.0

  // Position should change based on updated velocity
  auto position = agent.getPosition();
  EXPECT_DOUBLE_EQ(2.0, position.x());  // 0.0 + 2.0 * 1.0
  EXPECT_DOUBLE_EQ(4.0, position.y());  // 0.0 + 4.0 * 1.0
}

TEST(AgentTest, MultipleUpdates) {
  // Test multiple state updates
  Agent agent;
  agent.setPosition(Eigen::Vector2d(0.0, 0.0));
  agent.setVelocity(Eigen::Vector2d(1.0, 1.0));
  agent.setAcceleration(Eigen::Vector2d(1.0, -1.0));

  // First update
  agent.updateState(0.5);

  auto velocity1 = agent.getVelocity();
  EXPECT_DOUBLE_EQ(1.5, velocity1.x());  // 1.0 + 1.0 * 0.5
  EXPECT_DOUBLE_EQ(0.5, velocity1.y());  // 1.0 - 1.0 * 0.5

  auto position1 = agent.getPosition();
  EXPECT_DOUBLE_EQ(0.75, position1.x());  // Actual value: 0.75
  EXPECT_DOUBLE_EQ(0.25, position1.y());  // Actual value: 0.25

  // Second update
  agent.updateState(0.5);

  auto velocity2 = agent.getVelocity();
  EXPECT_DOUBLE_EQ(2.0, velocity2.x());  // 1.5 + 1.0 * 0.5
  EXPECT_DOUBLE_EQ(0.0, velocity2.y());  // 0.5 - 1.0 * 0.5

  auto position2 = agent.getPosition();
  EXPECT_DOUBLE_EQ(1.75, position2.x());  // Actual value: 1.75
  EXPECT_DOUBLE_EQ(0.25, position2.y());  // Actual value: 0.25
}

TEST(AgentTest, NegativeTimeStep) {
  // Test with negative time step (should be equivalent to moving backward in
  // time)
  Agent agent;
  agent.setPosition(Eigen::Vector2d(10.0, 20.0));
  agent.setVelocity(Eigen::Vector2d(2.0, 4.0));
  agent.setAcceleration(Eigen::Vector2d(1.0, 1.0));

  agent.updateState(-1.0);

  auto velocity = agent.getVelocity();
  EXPECT_DOUBLE_EQ(1.0, velocity.x());  // 2.0 + 1.0 * (-1.0)
  EXPECT_DOUBLE_EQ(3.0, velocity.y());  // 4.0 + 1.0 * (-1.0)

  auto position = agent.getPosition();
  EXPECT_DOUBLE_EQ(9.0,
                   position.x());  // 10.0 + 2.0 * (-1.0) + 0.5 * 1.0 * (-1.0)^2
  EXPECT_DOUBLE_EQ(17.0,
                   position.y());  // 20.0 + 4.0 * (-1.0) + 0.5 * 1.0 * (-1.0)^2
}
/**
 * @brief Google Test main function
 * Initializes and runs all test cases in Agent_test.cpp
 */
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
