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

  auto waypointFollowing = WaypointFollowing();
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
    auto accelerations = waypointFollowing.generateAccelerations(agents);

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
