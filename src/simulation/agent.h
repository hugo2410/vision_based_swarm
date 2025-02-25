//
// Copyright 2025 Hugo Birch
// created by hugo on 2/25/25.
//

#ifndef SRC_SIMULATION_AGENT_H_
#define SRC_SIMULATION_AGENT_H_
#include <utility>
#include <vector>

class Agent {
 public:
  Agent();
  void setPosition(const std::pair<double, double>& position);
  std::pair<double, double> getPosition() const;
  void updateState(const std::vector<int>& fov, double timeStep);
  std::pair<double, double> getAcceleration() const;

 private:
  std::pair<double, double> position;
  std::pair<double, double> velocity;
  std::pair<double, double> acceleration;
};

#endif  // SRC_SIMULATION_AGENT_H_
