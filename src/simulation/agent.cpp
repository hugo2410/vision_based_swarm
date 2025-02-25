//
// Copyright 2025 Hugo Birch
// created by hugo on 2/25/25.
//

#include "simulation/agent.h"

#include <utility>
#include <vector>

Agent::Agent()
    : position({0.0, 0.0}), velocity({0.0, 0.0}), acceleration({0.0, 0.0}) {}

void Agent::setPosition(const std::pair<double, double>& pos) {
  position = pos;
}

std::pair<double, double> Agent::getPosition() const { return position; }

void Agent::updateState(const std::vector<int>& fov, double timeStep) {
  // Compute new acceleration based on FOV and other rules.
  // For now, set a placeholder acceleration.
  acceleration = {0.1 * fov.size(), -0.1 * fov.size()};

  // Update velocity and position.
  velocity.first += acceleration.first * timeStep;
  velocity.second += acceleration.second * timeStep;

  position.first += velocity.first * timeStep;
  position.second += velocity.second * timeStep;
}

std::pair<double, double> Agent::getAcceleration() const {
  return acceleration;
}
