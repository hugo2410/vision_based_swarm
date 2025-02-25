// Copyright 2025 Hugo Birch
// Created by Hugo on 2/25/25.

#ifndef SRC_SIMULATION_AGENT_H_
#define SRC_SIMULATION_AGENT_H_

#include <utility>
#include <vector>

/**
 * @class Agent
 * @brief Represents an individual agent in a 2D/3D simulation.
 *
 * The Agent class models an entity in the simulation with position, velocity,
 * and acceleration. It provides methods to update its state based on external
 * inputs such as acceleration and time step.
 */
class Agent {
 public:
  /**
   * @brief Default constructor for the Agent class.
   *
   * Initializes the agent's position, velocity, and acceleration to zero.
   */
  Agent();

  /**
   * @brief Sets the position of the agent.
   *
   * @param position A pair representing the (x, y) coordinates of the agent.
   */
  void setPosition(const std::pair<double, double>& position);

  /**
   * @brief Gets the current position of the agent.
   *
   * @return A pair representing the (x, y) coordinates of the agent.
   */
  std::pair<double, double> getPosition() const;

  /**
   * @brief Updates the state of the agent based on acceleration and time step.
   *
   * This method updates the agent's velocity and position using its current
   * acceleration and a given time step.
   *
   * @param acceleration A vector of integers representing acceleration values
   *                     (e.g., visual field effects or external forces).
   * @param time_step The time step used for updating the state (in seconds).
   */
  void updateState(const std::vector<int>& acceleration, double time_step);

  /**
   * @brief Gets the current acceleration of the agent.
   *
   * @return A pair representing the (x, y) components of the agent's
   * acceleration.
   */
  std::pair<double, double> getAcceleration() const;

 private:
  /**
   * @brief The current position of the agent in 2D space (x, y).
   */
  std::pair<double, double> position;

  /**
   * @brief The current velocity of the agent in 2D space (vx, vy).
   */
  std::pair<double, double> velocity;

  /**
   * @brief The current acceleration of the agent in 2D space (ax, ay).
   */
  std::pair<double, double> acceleration;
};

#endif  // SRC_SIMULATION_AGENT_H_
