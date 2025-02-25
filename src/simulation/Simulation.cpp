//
// Copyright 2025 Hugo Birch
// created by hugo on 2/25/25.
//

#include "simulation/Simulation.h"

#include <vector>

Simulation::Simulation(int numAgents, double timeStep) : timeStep(timeStep) {
  agents.resize(numAgents);
}

void Simulation::update() {
  for (auto& agent : agents) {
    agent.updateState(timeStep);
  }
}

void Simulation::setAgentPositions(
    const std::vector<std::pair<double, double>>& positions) {
  for (size_t i = 0; i < positions.size(); ++i) {
    agents[i].setPosition(positions[i]);
  }
}

std::vector<std::pair<double, double>> Simulation::getAgentAccelerations()
    const {
  std::vector<std::pair<double, double>> accelerations;
  for (const auto& agent : agents) {
    accelerations.push_back(agent.getAcceleration());
  }
  return accelerations;
}
