// Copyright 2025 Hugo Birch
// Created by Hugo on 2/25/25.

#ifndef SRC_SIMULATION_SIMULATION_H_
#define SRC_SIMULATION_SIMULATION_H_

#include <Eigen/Dense>
#include <random>
#include <vector>

#include "simulation/Agent.h"
#include "simulation/SimulationParams.h"

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
   * @brief Constructor
   * @param params Configuration parameters for the simulation
   */
  explicit Simulation(const SimulationParams& params);

  /**
   * @brief Initialize all agents with starting positions and velocities
   */
  void initialize();

  /**
   * @brief Update all agents for a single time step using current accelerations
   */
  void update();

  /**
   * @brief Set custom accelerations for agents from external source
   * @param accelerations Vector of acceleration pairs for each agent
   */
  void setAgentAccelerations(const std::vector<Eigen::Vector2d>& accelerations);

  /**
   * @brief Get a reference to all agents
   * @return Reference to the vector of agents
   */
  std::vector<Agent>& getAgents();

  /**
   * @brief Get the trajectory history for all agents
   * @return Const reference to the trajectory data
   */
  const std::vector<std::vector<Eigen::Vector2d>>& getTrajectories() const;

  /**
   * @brief Get the current simulation time
   * @return The current time in simulation units
   */
  double getCurrentTime() const;

  /**
   * @brief Record current agent positions in trajectory history
   */
  void recordTrajectories();

 private:
  /**
   * @brief Initialize agents in a grid pattern
   */
  void initializeGrid();

  /**
   * @brief Initialize agents with random positions
   */
  void initializeRandom();

  SimulationParams params;    ///< Configuration parameters
  std::vector<Agent> agents;  ///< Collection of all agents
  std::mt19937 rng;           ///< Random number generator
  double currentTime;         ///< Current simulation time

  /**
   * @brief Stores agent trajectories over time for plotting
   *
   * Format: trajectories[agent_idx][time_step] = (x,y)
   */
  std::vector<std::vector<Eigen::Vector2d>> trajectories;
};

#endif  // SRC_SIMULATION_SIMULATION_H_
