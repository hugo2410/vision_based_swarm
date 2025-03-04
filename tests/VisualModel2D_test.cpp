// Copyright 2025 Hugo Birch
// Created by hugo on 3/4/25.
//

#include "visual_model/VisualModel2D.h"

#include <gtest/gtest.h>

#include <vector>

/**
 * @brief Test fixture for VisualModel2D.
 */
class VisualModel2DTest : public ::testing::Test {
 protected:
  VisualModel2D* visual_model;

  void SetUp() override {
    visual_model = new VisualModel2D(
        16, M_PI / 2, 0.1, 1.5, 0.0,  // nPhi, phi_max, R, v0, yaw0
        0.1, 0.1, 2.0, 1.0);          // drag, ang_drag, acc_max, yaw_rate_max
  }

  void TearDown() override { delete visual_model; }
};

/**
 * @brief Test conversion from forward acceleration to Cartesian acceleration.
 */
TEST_F(VisualModel2DTest, ComputeLinearAcceleration) {
  double forward_accel = 1.0;
  double yaw = M_PI / 4;  // 45 degrees
  double dt = 0.1;

  Eigen::Vector2d acc =
      visual_model->computeLinearAcceleration(forward_accel, yaw, dt);

  EXPECT_NEAR(acc.x(), std::cos(M_PI / 4), 0.001);
  EXPECT_NEAR(acc.y(), std::sin(M_PI / 4), 0.001);
}

/**
 * @brief Test computation of visual commands for agents.
 */
TEST_F(VisualModel2DTest, ComputeVisualCommands) {
  std::vector<VisualModel2D::AgentState> agents = {
      {{0.0, 0.0}, 1.0, 0.0},       // Agent at origin, facing right
      {{1.0, 1.0}, 1.0, M_PI / 4},  // Diagonal position
      {{-1.0, 0.0}, 1.0, M_PI}      // Facing left
  };

  double Vu = -1.0, Vp = -1.0, Vuu = 0.5, Vpp = 0.5, dVu = 0.2, dVp = 0.2;
  auto commands =
      visual_model->computeVisualCommands(agents, Vu, Vp, Vuu, Vpp, dVu, dVp);

  ASSERT_EQ(commands.size(), agents.size());

  for (const auto& cmd : commands) {
    EXPECT_LE(std::abs(cmd.forward_acceleration),
              2.0);                          // Within acceleration limits
    EXPECT_LE(std::abs(cmd.yaw_rate), 1.0);  // Within yaw rate limits
  }
}

/**
 * @brief Test handling of edge cases such as zero agents.
 */
TEST_F(VisualModel2DTest, HandlesEmptyAgents) {
  std::vector<VisualModel2D::AgentState> agents;
  auto commands = visual_model->computeVisualCommands(agents, -1.0, -1.0, 0.5,
                                                      0.5, 0.2, 0.2);

  EXPECT_EQ(commands.size(), 0);
}

/**
 * @brief Test limiting of acceleration values.
 */
TEST_F(VisualModel2DTest, LimitsAcceleration) {
  std::vector<VisualModel2D::AgentState> agents = {{{0.0, 0.0}, 1.0, 0.0}};

  auto commands = visual_model->computeVisualCommands(agents, -1.0, -1.0, 10.0,
                                                      10.0, 10.0, 10.0);

  EXPECT_EQ(commands.size(), 1);
  EXPECT_NEAR(commands[0].forward_acceleration, 2.0,
              0.001);                             // Limited to acc_max
  EXPECT_NEAR(commands[0].yaw_rate, 1.0, 0.001);  // Limited to yaw_rate_max
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
