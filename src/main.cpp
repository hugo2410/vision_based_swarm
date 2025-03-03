//
// Copyright 2025 Hugo Birch
// created by hugo on 2/25/25.
//
/**
 * @file main.cpp
 * @brief Main entry point for the vision-based swarm simulation
 * @author Hugo Birch
 * @date 2025-02-26
 */

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <utility>
#include <vector>

#include "simulation/Agent.h"
#include "simulation/Plotter.h"
#include "simulation/Simulation.h"
#include "simulation/SimulationParams.h"

/**
 * @brief Generate example accelerations for testing (will be replaced by
 * external library)
 * @param agents Reference to the current agents
 * @return Vector of acceleration pairs for each agent
 */
/**
 * @brief Generate accelerations for perfect circular motion around a point
 * @param agents Reference to the current agents
 * @return Vector of acceleration pairs for each agent
 */
std::vector<std::pair<double, double>> generateAccelerations(
    const std::vector<Agent>& agents) {
  std::vector<std::pair<double, double>> accelerations;

  // Define waypoints (x, y coordinates)
  static const std::vector<std::pair<double, double>> waypoints = {
      {20.0, 20.0},  // Bottom left
      {80.0, 20.0},  // Bottom right
      {80.0, 80.0},  // Top right
      {20.0, 80.0}   // Top left
  };

  // Keep track of current waypoint index for each agent
  static std::vector<int> currentWaypointIndices;

  // Initialize on first call
  if (currentWaypointIndices.size() != agents.size()) {
    currentWaypointIndices.resize(agents.size(), 0);
  }

  // Parameters for the controller
  double arrivalDistance = 2.0;  // How close is "arrived" at waypoint
  double maxSpeed = 10.0;        // Maximum desired speed
  double timeStep = 0.01;        // Should match your simulation timestep
  double maxAccel = 10.0;        // Maximum acceleration magnitude

  for (size_t i = 0; i < agents.size(); i++) {
    const auto& agent = agents[i];
    auto pos = agent.getPosition();
    auto vel = agent.getVelocity();

    // Get current waypoint index for this agent
    int& waypointIdx = currentWaypointIndices[i];
    const auto& targetPos = waypoints[waypointIdx];

    // Vector to waypoint
    double dx = targetPos.first - pos.first;
    double dy = targetPos.second - pos.second;
    double distToWaypoint = std::sqrt(dx * dx + dy * dy);

    // Check if we've reached the waypoint
    if (distToWaypoint < arrivalDistance) {
      // Move to next waypoint
      waypointIdx = (waypointIdx + 1) % waypoints.size();
      const auto& newTarget = waypoints[waypointIdx];

      // Recalculate vector to new waypoint
      dx = newTarget.first - pos.first;
      dy = newTarget.second - pos.second;
      distToWaypoint = std::sqrt(dx * dx + dy * dy);
    }

    // Calculate desired velocity
    double desiredVx = 0.0;
    double desiredVy = 0.0;

    if (distToWaypoint > 0.001) {
      // Normalize direction vector
      double dirX = dx / distToWaypoint;
      double dirY = dy / distToWaypoint;

      // Set velocity magnitude based on distance to waypoint
      // (slow down as we approach the waypoint)
      double speedFactor = std::min(1.0, distToWaypoint / 10.0);
      double speed = maxSpeed * speedFactor;

      // Desired velocity vector
      desiredVx = dirX * speed;
      desiredVy = dirY * speed;
    }

    // Calculate required acceleration to achieve desired velocity
    double ax = (desiredVx - vel.first) / timeStep;
    double ay = (desiredVy - vel.second) / timeStep;

    // Limit acceleration magnitude
    double accelMag = std::sqrt(ax * ax + ay * ay);
    if (accelMag > maxAccel) {
      ax = (ax / accelMag) * maxAccel;
      ay = (ay / accelMag) * maxAccel;
    }

    accelerations.push_back({ax, ay});
  }

  return accelerations;
}

int main() {
  // Create simulation parameters with defaults
  SimulationParams params;

  // Customize parameters
  params.numAgents = 1;
  params.timeStep = 0.01;
  params.simulationTime = 2000.0;
  params.worldSizeX = 100.0;
  params.worldSizeY = 100.0;
  params.maxInitialVelocity = 10.0;
  params.maxInitialAcceleration = 0.0;  // Start with zero acceleration
  params.initPositionType = "grid";     // or "random"

  // Create and initialize simulation
  std::cout << "Creating simulation with " << params.numAgents << " agents"
            << std::endl;
  Simulation sim(params);

  std::cout << "Initializing agents" << std::endl;
  sim.initialize();

  // Main simulation loop - this will interface with external libraries in the
  // future
  std::cout << "Running simulation for " << params.simulationTime << " seconds"
            << std::endl;

  int totalSteps = static_cast<int>(params.simulationTime / params.timeStep);
  int progressInterval =
      std::max(1, totalSteps / 20);  // Show progress ~20 times

  for (int step = 0; step < totalSteps; ++step) {
    // Get current agents
    auto& agents = sim.getAgents();

    // Get accelerations from external source - placeholder for now
    auto accelerations = generateAccelerations(agents);

    // Update agent accelerations
    sim.setAgentAccelerations(accelerations);

    // Update simulation state
    sim.update();

    // Display progress periodically
    if (step % progressInterval == 0) {
      double progress = 100.0 * step / totalSteps;
      std::cout << "Simulation progress: " << progress << "%" << std::endl;
    }
  }

  // Plot results
  std::cout << "Plotting results" << std::endl;
  Plotter plotter(sim, params);
  plotter.plotTrajectories("trajectories.png");
  plotter.createVideo("simulation.mp4", 60);

  std::cout << "Simulation completed successfully" << std::endl;

  return 0;
}
