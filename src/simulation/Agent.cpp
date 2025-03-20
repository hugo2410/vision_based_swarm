//
// Copyright 2025 Hugo Birch
// created by hugo on 2/25/25.
//

#include "Agent.h"

#include <Eigen/Dense>
#include <utility>
#include <vector>

Agent::Agent()
    : position(Eigen::Vector2d::Zero()),
      velocity(Eigen::Vector2d::Zero()),
      acceleration(Eigen::Vector2d::Zero()) {}

void Agent::setPosition(const Eigen::Vector2d& pos) { position = pos; }

void Agent::setVelocity(const Eigen::Vector2d& vel) { velocity = vel; }

void Agent::setAcceleration(const Eigen::Vector2d& acc) { acceleration = acc; }

Eigen::Vector2d Agent::getPosition() const { return position; }

Eigen::Vector2d Agent::getVelocity() const { return velocity; }

Eigen::Vector2d Agent::getAcceleration() const { return acceleration; }

void Agent::updateState(double timeStep) {
  // Update velocity and position using the agent's internal acceleration
  velocity += acceleration * timeStep;
  position += velocity * timeStep;
}
