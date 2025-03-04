// Copyright 2025 Hugo Birch
// Created by hugo on 3/4/25.
//
#include "simulation/WaypointFollowing.h"

#include <gtest/gtest.h>

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
      agents[i].setPosition({i * 10.0, i * 10.0});
      agents[i].setVelocity({0.0, 0.0});
    }
  }

  std::vector<Agent> agents;
};

/**
 * @brief Test default waypoint behavior
 */
TEST_F(WaypointFollowingTest, DefaultWaypoints) {
  WaypointFollowing controller;

  auto accelerations = controller.generateAccelerations(agents);
  ASSERT_EQ(accelerations.size(), agents.size());

  for (const auto& accel : accelerations) {
    EXPECT_LE(std::abs(accel.first),
              10.0);  // Should not exceed max acceleration
    EXPECT_LE(std::abs(accel.second), 10.0);
  }
}

/**
 * @brief Test custom waypoints
 */
TEST_F(WaypointFollowingTest, CustomWaypoints) {
  std::vector<std::pair<double, double>> customWaypoints = {
      {0.0, 0.0}, {50.0, 50.0}, {100.0, 100.0}};
  WaypointFollowing controller(customWaypoints);

  auto accelerations = controller.generateAccelerations(agents);
  ASSERT_EQ(accelerations.size(), agents.size());
}

/**
 * @brief Test if waypoints correctly update
 */
TEST_F(WaypointFollowingTest, WaypointSwitching) {
  WaypointFollowing controller;

  // Move an agent near a waypoint
  agents[0].setPosition({20.0, 20.0});  // Close to default first waypoint

  auto accelerations1 = controller.generateAccelerations(agents);
  EXPECT_NEAR(accelerations1[0].first, 0.0, 0.1);
  EXPECT_NEAR(accelerations1[0].second, 0.0, 0.1);

  // Move the agent further
  agents[0].setPosition({21.0, 21.0});
  auto accelerations2 = controller.generateAccelerations(agents);
  EXPECT_NE(accelerations1[0], accelerations2[0]);  // Should change waypoint
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
