// Copyright 2025 Hugo Birch
// Created by Hugo on 2/25/25.

#ifndef SRC_SIMULATION_SIMULATION_H_
#define SRC_SIMULATION_SIMULATION_H_

#include <vector>

#include "simulation/Agent.h"

/**
 * @class Simulation
 * @brief Manages the simulation of a swarm of agents.
 *
 * The Simulation class handles the state updates of all agents in the system
 * based on a given time step. It provides methods to set agent positions,
 * update their states, and retrieve their accelerations.
 */
class Simulation {
 public:
  /**
   * @brief Constructs a Simulation object.
   *
   * Initializes the simulation with a specified number of agents and a time
   * step.
   *
   * @param numAgents The number of agents in the simulation.
   * @param timeStep The time step for updating the simulation (in seconds).
   */
  Simulation(int numAgents, double timeStep);

  /**
   * @brief Updates the state of all agents in the simulation.
   *
   * This method updates each agent's position, velocity, and acceleration
   * based on their interactions and the simulation rules.
   */
  void update();

  /**
   * @brief Sets the positions of all agents in the simulation.
   *
   * @param positions A vector of pairs representing the (x, y) coordinates
   *                  for each agent.
   */
  void setAgentPositions(
      const std::vector<std::pair<double, double>>& positions);

  /**
   * @brief Retrieves the accelerations of all agents in the simulation.
   *
   * @return A vector of pairs representing the (x, y) components of
   * acceleration for each agent.
   */
  std::vector<std::pair<double, double>> getAgentAccelerations() const;

 private:
  /**
   * @brief A vector containing all agents in the simulation.
   */
  std::vector<Agent> agents;

  /**
   * @brief The time step used for updating the simulation (in seconds).
   */
  double timeStep;
};

#endif  // SRC_SIMULATION_SIMULATION_H_
