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
#include "simulation/WaypointFollowing.h"
#include "visual_model/VisualModel2D.h"

int main() {
  // Create simulation parameters with defaults
  SimulationParams params;

  // Customize parameters
  params.numAgents = 1;
  params.timeStep = 0.01;
  params.simulationTime = 2000.0;
  params.maxInitialVelocity = 10.0;
  params.maxInitialAcceleration = 0.0;  // Start with zero acceleration
  params.initPositionType = "grid";     // or "random"

  // Initialize visual model with parameters from SimulationParams
  auto visual_model = VisualModel2D(
      params.nPhi, params.phi_max, params.R, params.v0, params.yaw0,
      params.drag, params.ang_drag, params.acc_max, params.yaw_rate_max);

  // Create waypoint following instance for waypoint management
  auto waypoint_following =
      WaypointFollowing<Eigen::Vector2d>(params.waypoints, params);

  // Use waypoints from SimulationParams
  int current_waypoint = 0;

  // Create and initialize simulation
  std::cout << "Creating simulation with " << params.numAgents << " agents"
            << std::endl;
  Simulation sim(params);

  std::cout << "Initializing agents" << std::endl;
  sim.initialize();

  // Main simulation loop
  std::cout << "Running simulation for " << params.simulationTime << " seconds"
            << std::endl;

  int totalSteps = static_cast<int>(params.simulationTime / params.timeStep);
  int progressInterval =
      std::max(1, totalSteps / 20);  // Show progress ~20 times

  for (int step = 0; step < totalSteps; ++step) {
    auto& sim_agents = sim.getAgents();

    // Check if current waypoint is reached
    if (waypoint_following.isWaypointReached(sim_agents,
                                             params.waypoints[current_waypoint],
                                             params.waypointReachDistance)) {
      current_waypoint = (current_waypoint + 1) % params.waypoints.size();
    }

    // Convert agents to visual model format
    std::vector<VisualModel2D::AgentState> visual_agents;
    visual_agents.reserve(sim_agents.size());

    for (const auto& agent : sim_agents) {
      VisualModel2D::AgentState visual_state;
      auto pos = agent.getPosition();
      visual_state.position = Eigen::Vector2d(pos.x(), pos.y());
      auto vel = agent.getVelocity();
      visual_state.speed = Eigen::Vector2d(vel.x(), vel.y()).norm();
      visual_state.heading = std::atan2(vel.y(), vel.x());
      visual_agents.push_back(visual_state);
    }

    // Get control commands using current waypoint
    auto commands = visual_model.computeVisualCommands(
        visual_agents, -1.0, -1.0, 0.5, 0.5, 0.2, 0.2, 0,
        params.waypoints[current_waypoint]);
    std::cout << "Commands: " << commands[0].forward_acceleration << ", "
              << commands[0].yaw_rate << std::endl;
    // Convert commands to accelerations
    std::vector<Eigen::Vector2d> accelerations;
    accelerations.reserve(commands.size());

    for (size_t i = 0; i < commands.size(); ++i) {
      Eigen::Vector2d acc = visual_model.computeLinearAcceleration(
          commands[i].forward_acceleration, commands[i].yaw_rate);
      accelerations.push_back(acc);
    }

    // Update agent accelerations
    sim.setAgentAccelerations(accelerations);
    std::cout << "Accelerations: " << accelerations[0].x() << ", "
              << accelerations[0].y() << std::endl;

    // Update simulation state
    sim.update();
    std::cout << "Velocity: " << sim.getAgents()[0].getVelocity().x() << ", "
              << sim.getAgents()[0].getVelocity().y() << std::endl;

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
