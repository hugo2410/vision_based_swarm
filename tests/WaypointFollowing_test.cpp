// Copyright 2025 Hugo Birch
// Created by hugo on 3/4/25.
//
#include "simulation/WaypointFollowing.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <vector>

#include "simulation/Agent.h"

/**
 * @brief Test fixture for WaypointFollowing
 */
class WaypointFollowingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    agents = {Agent(), Agent(), Agent()};
    for (size_t i = 0; i < agents.size(); ++i) {
      agents[i].setPosition(Eigen::Vector2d(i * 10.0, i * 10.0));
      agents[i].setVelocity(Eigen::Vector2d(0.0, 0.0));
    }
  }

  std::vector<Agent> agents;
};

/**
 * @brief Test default waypoint behavior
 */
TEST_F(WaypointFollowingTest, DefaultWaypoints) {
  WaypointFollowing<Eigen::Vector2d> controller;

  auto accelerations = controller.generateAccelerations(agents);
  ASSERT_EQ(accelerations.size(), agents.size());

  for (const auto& accel : accelerations) {
    EXPECT_LE(accel.norm(), 10.0);  // Should not exceed max acceleration
  }
}

/**
 * @brief Test custom waypoints
 */
TEST_F(WaypointFollowingTest, CustomWaypoints) {
  std::vector<Eigen::Vector2d> customWaypoints = {
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(50.0, 50.0),
      Eigen::Vector2d(100.0, 100.0)};
  WaypointFollowing<Eigen::Vector2d> controller(customWaypoints);

  auto accelerations = controller.generateAccelerations(agents);
  ASSERT_EQ(accelerations.size(), agents.size());
}

/**
 * @brief Test if waypoints correctly update
 */
TEST_F(WaypointFollowingTest, WaypointSwitching) {
  WaypointFollowing<Eigen::Vector2d> controller;

  // Move an agent near a waypoint
  agents[0].setPosition(
      Eigen::Vector2d(19.0, 19.0));  // Close to default first waypoint

  auto accelerations1 = controller.generateAccelerations(agents);

  // Move the agent further
  agents[0].setPosition(Eigen::Vector2d(20.0, 20.0));
  auto accelerations2 = controller.generateAccelerations(agents);
  EXPECT_LE(accelerations1[0].norm(), accelerations2[0].norm());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
