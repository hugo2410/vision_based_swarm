// Copyright 2025 Hugo Birch
// Created by hugo on 3/4/25.
//

#ifndef SRC_VISUAL_MODEL_VISUALMODEL2D_H_
#define SRC_VISUAL_MODEL_VISUALMODEL2D_H_

#include <Eigen/Dense>
#include <cmath>
#include <utility>
#include <vector>

class VisualModel2D {
 public:
  struct AgentState {
    Eigen::Vector2d position;  // (x, y)
    double speed;              // magnitude of velocity
    double heading;            // angle in radians
  };

  struct ControlOutput {
    double forward_acceleration;
    double yaw_rate;
  };

  VisualModel2D(int nPhi, double phi_max, double R, double v0, double yaw0,
                double drag, double ang_drag, double acc_max,
                double yaw_rate_max);

  std::vector<ControlOutput> computeVisualCommands(
      const std::vector<AgentState>& agents, double Vu, double Vp, double Vuu,
      double Vpp, double dVu, double dVp, int focal_agent = -1,
      Eigen::Vector2d next_wpt = Eigen::Vector2d::Zero());

  Eigen::Vector2d computeLinearAcceleration(double forward_acceleration,
                                            double yaw, double dt);

 private:
  int nPhi;
  double phi_max;
  double R;
  double v0;
  double yaw0;
  double drag;
  double ang_drag;
  double acc_max;
  double yaw_rate_max;
  std::vector<double> Vc, Vs;
  double dPhi;

  void generateVisualFunction();
  Eigen::Matrix2d parameterVision2D(const std::vector<double>& V,
                                    const std::vector<double>& dV);
  std::pair<std::vector<double>, std::vector<int>> computeVisualField(
      const std::vector<AgentState>& agents, int i);
};

#endif  // SRC_VISUAL_MODEL_VISUALMODEL2D_H_
