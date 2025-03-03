//
// Copyright 2025 Hugo Birch
// created by hugo on 2/25/25.
//

#include "Agent.h"

#include <utility>
#include <vector>

Agent::Agent()
    : position({0.0, 0.0}), velocity({0.0, 0.0}), acceleration({0.0, 0.0}) {}

void Agent::setPosition(const std::pair<double, double>& pos) {
  position = pos;
}

void Agent::setVelocity(const std::pair<double, double>& vel) {
  velocity = vel;
}

void Agent::setAcceleration(const std::pair<double, double>& acc) {
  acceleration = acc;
}

std::pair<double, double> Agent::getPosition() const { return position; }

std::pair<double, double> Agent::getVelocity() const { return velocity; }

std::pair<double, double> Agent::getAcceleration() const {
  return acceleration;
}

void Agent::updateState(double timeStep) {
  // Update velocity and position using the agent's internal acceleration
  velocity.first += acceleration.first * timeStep;
  velocity.second += acceleration.second * timeStep;

  position.first += velocity.first * timeStep;
  position.second += velocity.second * timeStep;
}
