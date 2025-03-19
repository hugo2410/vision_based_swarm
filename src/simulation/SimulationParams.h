//
// Copyright 2025 Hugo Birch
// Created by hugo on 2/26/25.
//
/**
 * @file SimulationParams.h
 * @brief Defines parameters class for simulation configuration
 * @author Hugo Birch
 * @date 2025-02-26
 */

#ifndef SRC_SIMULATION_SIMULATIONPARAMS_H_
#define SRC_SIMULATION_SIMULATIONPARAMS_H_

#include <Eigen/Dense>
#include <string>
#include <utility>
#include <vector>

/**
 * @class SimulationParams
 * @brief Contains all parameters for configuring the simulation
 *
 * This class centralizes all simulation parameters in one place for
 * easier management and configuration.
 */
class SimulationParams {
 public:
  // Simulation parameters
  int numAgents;
  double timeStep;
  double simulationTime;
  double worldSizeX;
  double worldSizeY;
  double maxInitialVelocity;
  double maxInitialAcceleration;
  std::string initPositionType;

  // Visual model parameters
  int nPhi;             // Number of angular divisions
  double phi_max;       // Maximum viewing angle
  double R;             // Agent radius
  double v0;            // Desired speed
  double yaw0;          // Desired heading
  double drag;          // Linear drag coefficient
  double ang_drag;      // Angular drag coefficient
  double acc_max;       // Maximum acceleration
  double yaw_rate_max;  // Maximum yaw rate

  // Waypoint parameters
  double
      waypointReachDistance;  // Distance threshold to consider waypoint reached
  std::vector<Eigen::Vector2d> waypoints;  // List of waypoints

  // Waypoint following parameters
  double arrivalDistance;  // Distance to consider waypoint reached
  double maxSpeed;         // Maximum speed for agents
  double maxAccel;         // Maximum acceleration magnitude

  /**
   * @brief Default constructor with reasonable default values
   */
  SimulationParams()
      : numAgents(100),
        timeStep(0.01),
        simulationTime(10.0),
        worldSizeX(100.0),
        worldSizeY(100.0),
        maxInitialVelocity(5.0),
        maxInitialAcceleration(1.0),
        initPositionType("grid"),
        // Visual model defaults
        nPhi(16),
        phi_max(M_PI / 2),
        R(0.3),
        v0(1.0),
        yaw0(0.0),
        drag(0.5),
        ang_drag(0.8),
        acc_max(2.0),
        yaw_rate_max(0.50),
        // Waypoint defaults
        waypointReachDistance(5.0),
        waypoints({Eigen::Vector2d(50.0, 50.0), Eigen::Vector2d(80.0, 80.0),
                   Eigen::Vector2d(20.0, 80.0), Eigen::Vector2d(50.0, 50.0)}),
        // Waypoint following defaults
        arrivalDistance(1.0),
        maxSpeed(10.0),
        maxAccel(10.0) {}
};

#endif  // SRC_SIMULATION_SIMULATIONPARAMS_H_
