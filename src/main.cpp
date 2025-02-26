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
std::vector<std::pair<double, double>> generateAccelerations(
    const std::vector<Agent>& agents) {
  // This is a placeholder function that will be replaced by your external
  // library For now, it creates some simple demo accelerations

  std::vector<std::pair<double, double>> accelerations;

  for (const auto& agent : agents) {
    // Example: Circular motion tendency
    auto pos = agent.getPosition();
    double centerX = 50.0;
    double centerY = 50.0;

    // Vector from agent to center
    double dx = centerX - pos.first;
    double dy = centerY - pos.second;

    // Normalize and scale
    double dist = std::sqrt(dx * dx + dy * dy);
    double scale = 0.5;  // Adjust strength of acceleration

    if (dist > 0.001) {
      // Add some circular component
      accelerations.push_back(
          {scale * (dy / dist),  // Perpendicular to radius for circular motion
           scale * (-dx / dist)});
    } else {
      accelerations.push_back({0.0, 0.0});
    }
  }

  return accelerations;
}

int main() {
  // Create simulation parameters with defaults
  SimulationParams params;

  // Customize parameters
  params.numAgents = 50;
  params.timeStep = 0.01;
  params.simulationTime = 20.0;
  params.worldSizeX = 100.0;
  params.worldSizeY = 100.0;
  params.maxInitialVelocity = 2.0;
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
