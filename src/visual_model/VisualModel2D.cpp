// Copyright 2025 Hugo Birch
// Created by hugo on 3/4/25.
//

#include "visual_model/VisualModel2D.h"

#include <algorithm>
#include <cmath>  // Include cmath for std::abs, std::round, etc.
#include <iostream>
#include <utility>
#include <vector>

VisualModel2D::VisualModel2D(int nPhi, double phi_max, double R, double v0,
                             double yaw0, double drag, double ang_drag,
                             double acc_max, double yaw_rate_max)
    : nPhi(nPhi),
      phi_max(phi_max),
      R(R),
      v0(v0),
      yaw0(yaw0),
      drag(drag),
      ang_drag(ang_drag),
      acc_max(acc_max),
      yaw_rate_max(yaw_rate_max) {
  generateVisualFunction();
}

void VisualModel2D::generateVisualFunction() {
  int s = 2 * nPhi + 1;
  Vc.resize(s);
  Vs.resize(s);

  dPhi = 2 * phi_max / (s - 1);

  for (int i = 0; i < s; i++) {
    double phi = -phi_max + i * dPhi;
    Vc[i] = std::cos(phi);
    Vs[i] = std::sin(phi);
  }
}

std::pair<std::vector<double>, std::vector<int>>
VisualModel2D::computeVisualField(const std::vector<AgentState>& agents,
                                  int i) {
  int V_size = 2 * nPhi + 1;
  std::vector<double> V(V_size, 0);
  std::vector<int> V_neig(V_size, -1);

  // Corrected: dPhi should be based on nPhi and phi_max, not 2*M_PI
  double dPhi_local =
      2 * phi_max / (V_size - 1);  // Consistent with generateVisualFunction

  for (size_t j = 0; j < agents.size(); j++) {
    if (static_cast<size_t>(i) == j) continue;

    Eigen::Vector2d rel_pos = agents[j].position - agents[i].position;
    double dist = rel_pos.norm();
    // Corrected: Angle calculation and normalization
    double angle_to_j =
        std::atan2(rel_pos.y(), rel_pos.x()) - agents[i].heading;
    angle_to_j = std::fmod(angle_to_j + M_PI, 2 * M_PI) -
                 M_PI;  // Normalize to [-pi, pi]

    // Corrected: Use phi_max and R to determine visibility
    if (std::abs(angle_to_j) <= phi_max + std::atan2(R, dist)) {
      // Corrected: Pixel calculation based on dPhi_local
      int px_phi_c = static_cast<int>(
          std::round((angle_to_j + phi_max) /
                     dPhi_local));  // Shift to [0, 2*phi_max] range
      int d_px = static_cast<int>(std::round(std::atan2(R, dist) / dPhi_local));

      int start = std::max(0, px_phi_c - d_px);
      int end = std::min(V_size - 1, px_phi_c + d_px);

      for (int p = start; p <= end; p++) {
        if (V[p] == 1) {
          int k = V_neig[p];
          if ((agents[i].position - agents[j].position).norm() <
              (agents[i].position - agents[k].position).norm()) {
            V_neig[p] = j;
          }
        } else {
          V_neig[p] = j;
        }
        V[p] = 1;
      }
    }
  }

  return {V, V_neig};
}

Eigen::Matrix2d VisualModel2D::parameterVision2D(
    const std::vector<double>& V, const std::vector<double>& dV) {
  Eigen::Matrix2d Vup = Eigen::Matrix2d::Zero();

  // Normalize by the total angle range
  double normalization = 2 * phi_max;

  for (size_t i = 0; i < V.size(); i++) {
    Vup(0, 0) += V[i] * Vc[i] * dPhi / normalization;
    Vup(0, 1) += V[i] * Vs[i] * dPhi / normalization;
    Vup(1, 0) += dV[i] * Vc[i] / normalization;
    Vup(1, 1) += dV[i] * Vs[i] / normalization;
  }
  return Vup;
}

std::vector<VisualModel2D::ControlOutput> VisualModel2D::computeVisualCommands(
    const std::vector<AgentState>& agents, double Vu, double Vp, double Vuu,
    double Vpp, double dVu, double dVp, [[maybe_unused]] int focal_agent,
    Eigen::Vector2d next_wpt) {
  std::vector<ControlOutput> dU(agents.size());
  bool use_waypoint = (next_wpt.norm() > 0);

  for (size_t i = 0; i < agents.size(); i++) {
    auto [V, V_neig] = computeVisualField(agents, i);
    std::vector<double> dV(V.size());
    for (size_t j = 0; j < V.size(); j++) {
      dV[j] =
          std::abs(V[(j + 1) % V.size()] - V[(j - 1 + V.size()) % V.size()]) /
          2;
    }

    Eigen::Matrix2d Vup = parameterVision2D(V, dV);

    // Calculate desired heading and yaw error
    double desired_heading = yaw0;  // Default desired heading
    double yaw_error = 0.0;

    if (use_waypoint) {
      Eigen::Vector2d rel_wpt = next_wpt - agents[i].position;

      // Calculate angle to waypoint
      desired_heading = std::atan2(rel_wpt.y(), rel_wpt.x());

      // Calculate yaw error (difference between desired and current heading)
      yaw_error = desired_heading - agents[i].heading;

      // Normalize to [-π, π] to ensure shortest turn direction
      while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
      while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
    }

    // Adjust the linear acceleration calculation
    double speed_error = v0 - agents[i].speed;
    double dU_lin =
        drag * speed_error + Vuu * (Vu * Vup(0, 0) + dVu * Vup(1, 0));

    double dU_ang = use_waypoint * ang_drag * yaw_error +
                    Vpp * (Vp * Vup(0, 1) + dVp * Vup(1, 1));

    dU_lin = std::clamp(dU_lin, -acc_max, acc_max);
    dU_ang = std::clamp(dU_ang, -yaw_rate_max, yaw_rate_max);

    dU[i] = {dU_lin, dU_ang};
  }

  return dU;
}

Eigen::Vector2d VisualModel2D::computeLinearAcceleration(
    double forward_acceleration, double yaw_rate, double current_heading,
    double current_speed) {
  // Compute the body-frame lateral acceleration due to turning.
  // It has magnitude (current_speed * yaw_rate)
  double lateral_acceleration = current_speed * yaw_rate;

  // Convert from body frame [forward; lateral] to world frame using a rotation
  // matrix. Body frame: x-axis is forward, y-axis is to the left.
  double acc_x_world = forward_acceleration * std::cos(current_heading) -
                       lateral_acceleration * std::sin(current_heading);
  double acc_y_world = forward_acceleration * std::sin(current_heading) +
                       lateral_acceleration * std::cos(current_heading);

  return Eigen::Vector2d(acc_x_world, acc_y_world);
}
