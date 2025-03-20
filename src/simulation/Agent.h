// Copyright 2025 Hugo Birch
// Created by Hugo on 2/25/25.

#ifndef SRC_SIMULATION_AGENT_H_
#define SRC_SIMULATION_AGENT_H_

#include <Eigen/Dense>
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
  void setPosition(const Eigen::Vector2d& position);

  /**
   * @brief Sets the velocity of the agent.
   *
   * @param vel A pair representing the (x, y) velocity of the agent.
   */
  void setVelocity(const Eigen::Vector2d& vel);

  /**
   * @brief Sets the acceleration of the agent.
   *
   * @param acc A pair representing the (x, y) acceleration of the agent.
   */
  void setAcceleration(const Eigen::Vector2d& acc);

  /**
   * @brief Gets the current position of the agent.
   *
   * @return A pair representing the (x, y) coordinates of the agent.
   */
  Eigen::Vector2d getPosition() const;

  /**
   * @brief Updates the state of the agent based on acceleration and time step.
   *
   * This method updates the agent's velocity and position using its current
   * acceleration and a given time step.
   *
   * @param time_step The time step used for updating the state (in seconds).
   */
  void updateState(double time_step);

  /**
   * @brief Gets the current acceleration of the agent.
   *
   * @return A pair representing the (x, y) components of the agent's
   * acceleration.
   */
  Eigen::Vector2d getAcceleration() const;

  /**
   * @brief Gets the current velocity of the agent.
   *
   * @return A pair representing the (x, y) components of the agent's
   * velocity.
   */
  Eigen::Vector2d getVelocity() const;

 private:
  /**
   * @brief The current position of the agent in 2D space (x, y).
   */
  Eigen::Vector2d position;

  /**
   * @brief The current velocity of the agent in 2D space (vx, vy).
   */
  Eigen::Vector2d velocity;

  /**
   * @brief The current acceleration of the agent in 2D space (ax, ay).
   */
  Eigen::Vector2d acceleration;
};

#endif  // SRC_SIMULATION_AGENT_H_
