// Copyright 2025 Hugo Birch
// Created by hugo on 3/4/25.
//

#include "visual_model/VisualModel2D.h"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

/**
 * @brief Test fixture for VisualModel2D.
 */
class VisualModel2DTest : public ::testing::Test {
 protected:
  std::unique_ptr<VisualModel2D> visual_model;

  void SetUp() override {
    // Using more realistic parameters
    visual_model = std::make_unique<VisualModel2D>(
        16,        // nPhi: number of angular divisions
        M_PI / 2,  // phi_max: maximum viewing angle (90 degrees)
        0.3,       // R: agent radius
        1.5,       // v0: desired speed
        0.0,       // yaw0: desired heading
        0.5,       // drag: linear drag coefficient
        0.8,       // ang_drag: angular drag coefficient
        2.0,       // acc_max: maximum acceleration
        1.0);      // yaw_rate_max: maximum yaw rate
  }
};

/**
 * @brief Test generation of visual function vectors.
 */
TEST_F(VisualModel2DTest, GenerateVisualFunction) {
  // Test the initialization of visual function vectors
  EXPECT_EQ(visual_model->getVc().size(), 33);  // 2 * nPhi + 1
  EXPECT_EQ(visual_model->getVs().size(), 33);

  // Test some key angles
  // First element should be at -phi_max (90 degrees or π/2)
  EXPECT_NEAR(visual_model->getVc()[0], std::cos(-M_PI / 2), 1e-6);
  EXPECT_NEAR(visual_model->getVs()[0], std::sin(-M_PI / 2), 1e-6);

  // Middle element should be at 0 degrees
  int middle_index = visual_model->getVc().size() / 2;
  EXPECT_NEAR(visual_model->getVc()[middle_index], 1.0, 1e-6);  // cos(0) = 1
  EXPECT_NEAR(visual_model->getVs()[middle_index], 0.0, 1e-6);  // sin(0) = 0

  // Last element should be at phi_max (90 degrees or π/2)
  int last_index = visual_model->getVc().size() - 1;
  EXPECT_NEAR(visual_model->getVc()[last_index], std::cos(M_PI / 2), 1e-6);
  EXPECT_NEAR(visual_model->getVs()[last_index], std::sin(M_PI / 2), 1e-6);
}

/**
 * @brief Test computation of visual field.
 */
TEST_F(VisualModel2DTest, ComputeVisualField) {
  std::vector<VisualModel2D::AgentState> agents = {
      {{0.0, 0.0}, 1.0, 0.0},  // Agent 0 at origin
      {{1.0, 0.0}, 1.0, 0.0},  // Agent 1 directly to the right
      {{0.0, 1.0}, 1.0, 0.0},  // Agent 2 directly above
      {{-2.0, 0.0}, 1.0, 0.0}  // Agent 3 outside visual range
  };

  // Test visual field from Agent 0's perspective
  auto [V, V_neig] = visual_model->computeVisualField(agents, 0);

  EXPECT_EQ(V.size(), 33);  // 2 * nPhi + 1
  EXPECT_EQ(V_neig.size(), 33);

  // Agent 1 should be visible (at 0 degrees)
  int center_index = V.size() / 2;
  EXPECT_EQ(V[center_index], 1);
  EXPECT_EQ(V_neig[center_index], 1);

  // Agent 3 should be outside visual range
  EXPECT_EQ(V[0], 0);
  EXPECT_EQ(V_neig[0], -1);
}

/**
 * @brief Test parameter vision 2D.
 */
TEST_F(VisualModel2DTest, ParameterVision2D) {
  std::vector<double> V(33, 0);  // 2 * nPhi + 1
  std::vector<double> dV(33, 0);

  // Set up a simple test case with one visible agent
  int center_index = V.size() / 2;
  V[center_index] = 1.0;
  dV[center_index] = 0.5;

  Eigen::Matrix2d Vup = visual_model->parameterVision2D(V, dV);

  // Check matrix properties
  EXPECT_GT(Vup(0, 0), 0);          // Should have positive x component
  EXPECT_NEAR(Vup(0, 1), 0, 1e-6);  // Should have near-zero y component
}

/**
 * @brief Test computation of visual commands for agents.
 */
TEST_F(VisualModel2DTest, ComputeVisualCommands) {
  std::vector<VisualModel2D::AgentState> agents = {
      {{0.0, 0.0}, 1.0, 0.0},       // Agent 0 at origin
      {{1.0, 0.0}, 1.0, M_PI / 4},  // Agent 1 to the right
      {{0.0, 1.0}, 1.0, -M_PI / 4}  // Agent 2 above
  };

  double Vu = -1.0;  // Repulsive linear coefficient
  double Vp = -1.0;  // Repulsive angular coefficient
  double Vuu = 0.5;  // Linear control gain
  double Vpp = 0.5;  // Angular control gain
  double dVu = 0.2;  // Linear derivative gain
  double dVp = 0.2;  // Angular derivative gain
  int focal_agent = 0;
  Eigen::Vector2d next_wpt(2.0, 2.0);  // Waypoint in positive quadrant

  auto commands = visual_model->computeVisualCommands(
      agents, Vu, Vp, Vuu, Vpp, dVu, dVp, focal_agent, next_wpt);

  ASSERT_EQ(commands.size(), agents.size());

  // Test command bounds
  for (const auto& cmd : commands) {
    EXPECT_LE(std::abs(cmd.forward_acceleration), visual_model->getAccMax());
    EXPECT_LE(std::abs(cmd.yaw_rate), visual_model->getYawRateMax());
  }

  // Test waypoint following behavior
  auto focal_cmd = commands[focal_agent];
  EXPECT_GT(focal_cmd.forward_acceleration,
            0);  // Should accelerate towards waypoint
}

/**
 * @brief Test computation of linear acceleration.
 */
TEST_F(VisualModel2DTest, ComputeLinearAcceleration) {
  // Test various angles
  std::vector<double> test_angles = {0, M_PI / 4, M_PI / 2, M_PI, -M_PI / 4};
  double forward_accel = 1.0;

  for (double yaw : test_angles) {
    Eigen::Vector2d acc =
        visual_model->computeLinearAcceleration(forward_accel, yaw);

    EXPECT_NEAR(acc.x(), forward_accel * std::cos(yaw), 1e-6);
    EXPECT_NEAR(acc.y(), forward_accel * std::sin(yaw), 1e-6);
    EXPECT_NEAR(acc.norm(), std::abs(forward_accel), 1e-6);
  }
}

/**
 * @brief Test handling of edge cases such as zero agents.
 */
TEST_F(VisualModel2DTest, HandlesEmptyAgents) {
  std::vector<VisualModel2D::AgentState> agents;
  auto commands = visual_model->computeVisualCommands(agents, -1.0, -1.0, 0.5,
                                                      0.5, 0.2, 0.2, 0);

  EXPECT_EQ(commands.size(), 0);
}

/**
 * @brief Test handling of multiple agents.
 */
TEST_F(VisualModel2DTest, HandlesMultipleAgents) {
  std::vector<VisualModel2D::AgentState> agents = {{{0.0, 0.0}, 1.0, 0.0},
                                                   {{2.0, 0.0}, 1.0, 0.0},
                                                   {{0.0, 2.0}, 1.0, 0.0},
                                                   {{-2.0, 0.0}, 1.0, 0.0}};

  auto commands = visual_model->computeVisualCommands(agents, -1.0, -1.0, 0.5,
                                                      0.5, 0.2, 0.2, 0);

  EXPECT_EQ(commands.size(), agents.size());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
