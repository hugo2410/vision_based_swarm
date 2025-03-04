// Copyright 2025 Hugo Birch
// Created by Hugo on 3/4/25.
//

#include "simulation/WaypointFollowing.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

/**
 * @brief Constructor with optional waypoints
 * @param waypoints Custom waypoints (optional, defaults will be used if empty)
 */
WaypointFollowing::WaypointFollowing(
    std::vector<std::pair<double, double>> waypoints) {
  // If no waypoints are provided, use defaults
  if (waypoints.empty()) {
    waypoints_ = {
        {20.0, 20.0},  // Bottom left
        {80.0, 20.0},  // Bottom right
        {80.0, 80.0},  // Top right
        {20.0, 80.0}   // Top left
    };
  } else {
    waypoints_ = std::move(waypoints);
  }
}

/**
 * @brief Generate example accelerations for agents following waypoints.
 * @param agents Reference to the current agents
 * @return Vector of acceleration pairs for each agent
 */
std::vector<std::pair<double, double>> WaypointFollowing::generateAccelerations(
    const std::vector<Agent>& agents) {
  std::vector<std::pair<double, double>> accelerations;

  // Initialize waypoint indices if first time running
  if (currentWaypointIndices_.size() != agents.size()) {
    currentWaypointIndices_.resize(agents.size(), 0);
  }

  // Controller parameters
  const double arrivalDistance =
      1.0;  // When the agent considers itself at the waypoint
  const double maxSpeed = 10.0;  // Maximum velocity
  const double timeStep = 0.01;  // Time step for simulation
  const double maxAccel = 10.0;  // Maximum acceleration magnitude

  for (size_t i = 0; i < agents.size(); i++) {
    const auto& agent = agents[i];
    auto pos = agent.getPosition();
    auto vel = agent.getVelocity();

    // Get current waypoint index for this agent
    int& waypointIdx = currentWaypointIndices_[i];
    const auto& targetPos = waypoints_[waypointIdx];

    // Compute vector to waypoint
    double dx = targetPos.first - pos.first;
    double dy = targetPos.second - pos.second;
    double distToWaypoint = std::sqrt(dx * dx + dy * dy);

    // Check if waypoint is reached
    if (distToWaypoint < arrivalDistance) {
      // Move to next waypoint (loop around)
      waypointIdx = (waypointIdx + 1) % waypoints_.size();
    }

    // Recalculate vector to new waypoint
    const auto& newTarget = waypoints_[waypointIdx];
    dx = newTarget.first - pos.first;
    dy = newTarget.second - pos.second;
    distToWaypoint = std::sqrt(dx * dx + dy * dy);

    // Compute desired velocity direction
    double desiredVx = 0.0;
    double desiredVy = 0.0;

    if (distToWaypoint > 0.001) {  // Avoid division by zero
      double dirX = dx / distToWaypoint;
      double dirY = dy / distToWaypoint;

      // Slow down as we approach the waypoint
      double speedFactor = std::min(1.0, distToWaypoint / 10.0);
      double speed = maxSpeed * speedFactor;

      // Desired velocity
      desiredVx = dirX * speed;
      desiredVy = dirY * speed;
    }

    // Compute required acceleration
    double ax = (desiredVx - vel.first) / timeStep;
    double ay = (desiredVy - vel.second) / timeStep;

    // Limit acceleration
    double accelMag = std::sqrt(ax * ax + ay * ay);
    if (accelMag > maxAccel) {
      ax = (ax / accelMag) * maxAccel;
      ay = (ay / accelMag) * maxAccel;
    }

    accelerations.push_back({ax, ay});
  }

  return accelerations;
}
