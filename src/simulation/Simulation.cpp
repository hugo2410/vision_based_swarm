//
// Copyright 2025 Hugo Birch
// created by hugo on 2/25/25.
//

#include "simulation/Simulation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

Simulation::Simulation(const SimulationParams& params)
    : params(params), currentTime(0.0) {
  // Seed random number generator
  std::random_device rd;
  rng = std::mt19937(rd());

  // Resize agents vector
  agents.resize(params.numAgents);

  // Initialize trajectories storage with one vector per agent
  trajectories.resize(params.numAgents);
}

void Simulation::initialize() {
  if (params.initPositionType == "grid") {
    initializeGrid();
  } else if (params.initPositionType == "random") {
    initializeRandom();
  } else {
    std::cerr << "Unknown initialization type: " << params.initPositionType
              << std::endl;
    std::cerr << "Defaulting to grid initialization" << std::endl;
    initializeGrid();
  }

  // Clear trajectory storage
  for (auto& traj : trajectories) {
    traj.clear();
  }

  // Reset current time
  currentTime = 0.0;

  // Record initial positions
  recordTrajectories();
}

void Simulation::initializeGrid() {
  int gridSize = static_cast<int>(std::ceil(std::sqrt(params.numAgents)));
  double cellWidth = params.worldSizeX / gridSize;
  double cellHeight = params.worldSizeY / gridSize;

  std::uniform_real_distribution<double> velDist(-params.maxInitialVelocity,
                                                 params.maxInitialVelocity);
  std::uniform_real_distribution<double> accDist(-params.maxInitialAcceleration,
                                                 params.maxInitialAcceleration);

  for (int i = 0; i < params.numAgents; ++i) {
    int row = i / gridSize;
    int col = i % gridSize;

    double x = (col + 0.5) * cellWidth;
    double y = (row + 0.5) * cellHeight;

    agents[i].setPosition({x, y});
    agents[i].setVelocity({velDist(rng), velDist(rng)});
    agents[i].setAcceleration({accDist(rng), accDist(rng)});
  }
}

void Simulation::initializeRandom() {
  std::uniform_real_distribution<double> xDist(0, params.worldSizeX);
  std::uniform_real_distribution<double> yDist(0, params.worldSizeY);
  std::uniform_real_distribution<double> velDist(-params.maxInitialVelocity,
                                                 params.maxInitialVelocity);
  std::uniform_real_distribution<double> accDist(-params.maxInitialAcceleration,
                                                 params.maxInitialAcceleration);

  for (int i = 0; i < params.numAgents; ++i) {
    agents[i].setPosition({xDist(rng), yDist(rng)});
    agents[i].setVelocity({velDist(rng), velDist(rng)});
    agents[i].setAcceleration({accDist(rng), accDist(rng)});
  }
}

void Simulation::update() {
  // Update each agent using its current acceleration
  for (auto& agent : agents) {
    agent.updateState(params.timeStep);
  }

  // Update current time
  currentTime += params.timeStep;

  // Record the updated positions
  recordTrajectories();
}

void Simulation::setAgentAccelerations(
    const std::vector<std::pair<double, double>>& accelerations) {
  // Update agent accelerations from external source
  if (accelerations.size() != agents.size()) {
    std::cerr << "Warning: acceleration vector size (" << accelerations.size()
              << ") doesn't match agent count (" << agents.size() << ")"
              << std::endl;
  }

  size_t count = std::min(accelerations.size(), agents.size());
  for (size_t i = 0; i < count; ++i) {
    agents[i].setAcceleration(accelerations[i]);
  }
}

void Simulation::recordTrajectories() {
  // Record current positions for trajectories
  for (size_t i = 0; i < agents.size(); ++i) {
    trajectories[i].push_back(agents[i].getPosition());
  }
}

std::vector<Agent>& Simulation::getAgents() { return agents; }

const std::vector<std::vector<std::pair<double, double>>>&
Simulation::getTrajectories() const {
  return trajectories;
}

double Simulation::getCurrentTime() const { return currentTime; }
