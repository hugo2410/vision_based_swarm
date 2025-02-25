//
// Copyright 2025 Hugo Birch
// created by hugo on 2/25/25.
//
#ifndef SRC_SIMULATION_SIMULATION_H_
#define SRC_SIMULATION_SIMULATION_H_

#include <vector>

#include "simulation/agent.h"

class Simulation {
 public:
  Simulation(int numAgents, double timeStep);
  void update();  // Updates the state of all agents in the simulation
  void setAgentPositions(
      const std::vector<std::pair<double, double>>& positions);
  std::vector<std::pair<double, double>> getAgentAccelerations() const;

 private:
  std::vector<Agent> agents;
  FOVCalculator fovCalculator;
  double timeStep;
};

#endif  // SRC_SIMULATION_SIMULATION_H_
