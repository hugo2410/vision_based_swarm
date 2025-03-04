// Copyright 2025 Hugo Birch
// Created by hugo on 3/4/25.
//

#include "visual_model/VisualModel2D.h"

#include <algorithm>
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

  double phi_step = (2 * M_PI) / s;
  dPhi = phi_step;

  for (int i = 0; i < s; i++) {
    double phi = -M_PI + i * phi_step;
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

  double dPhi = 2 * M_PI / V_size;

  for (size_t j = 0; j < agents.size(); j++) {
    if (i == j) continue;

    Eigen::Vector2d rel_pos = agents[j].position - agents[i].position;
    double dist = rel_pos.norm();
    double angle_to_j =
        std::atan2(rel_pos.y(), rel_pos.x()) - agents[i].heading;

    if (angle_to_j > M_PI) angle_to_j -= 2 * M_PI;
    if (angle_to_j < -M_PI) angle_to_j += 2 * M_PI;

    if (std::abs(angle_to_j) <= phi_max + std::atan2(R, dist)) {
      int px_phi_c = static_cast<int>(std::round((M_PI + angle_to_j) / dPhi));
      int d_px = static_cast<int>(std::round(std::atan2(R, dist) / dPhi));

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
  for (size_t i = 0; i < V.size(); i++) {
    Vup(0, 0) += V[i] * Vc[i] * dPhi;
    Vup(0, 1) += V[i] * Vs[i] * dPhi;
    Vup(1, 0) += dV[i] * Vc[i];
    Vup(1, 1) += dV[i] * Vs[i];
  }
  return Vup;
}

std::vector<VisualModel2D::ControlOutput> VisualModel2D::computeVisualCommands(
    const std::vector<AgentState>& agents, double Vu, double Vp, double Vuu,
    double Vpp, double dVu, double dVp, int focal_agent,
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

    double yaw0_i = yaw0;
    if (use_waypoint) {
      Eigen::Vector2d rel_wpt = next_wpt - agents[i].position;
      yaw0_i = std::atan2(rel_wpt.y(), rel_wpt.x());
    }

    double yaw_error = yaw0_i - agents[i].heading;
    yaw_error = std::fmod(yaw_error + M_PI, 2 * M_PI) - M_PI;

    double dU_lin = drag * (v0 - agents[i].speed) +
                    Vuu * (Vu * Vup(0, 0) + dVu * Vup(1, 0));
    double dU_ang = use_waypoint * ang_drag * yaw_error +
                    Vpp * (Vp * Vup(0, 1) + dVp * Vup(1, 1));

    dU_lin = std::clamp(dU_lin, -acc_max, acc_max);
    dU_ang = std::clamp(dU_ang, -yaw_rate_max, yaw_rate_max);

    dU[i] = {dU_lin, dU_ang};
  }

  return dU;
}

Eigen::Vector2d VisualModel2D::computeLinearAcceleration(
    double forward_acceleration, double yaw, double dt) {
  double ax = forward_acceleration * std::cos(yaw);
  double ay = forward_acceleration * std::sin(yaw);
  return Eigen::Vector2d(ax, ay);
}
