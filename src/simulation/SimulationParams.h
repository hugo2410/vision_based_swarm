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

#include <string>

/**
 * @class SimulationParams
 * @brief Contains all parameters for configuring the simulation
 *
 * This class centralizes all simulation parameters in one place for
 * easier management and configuration.
 */
class SimulationParams {
 public:
  int numAgents;                  ///< Number of agents in the simulation
  double timeStep;                ///< Time increment for each update step
  double simulationTime;          ///< Total time to run the simulation
  double worldSizeX;              ///< Width of the simulation world
  double worldSizeY;              ///< Height of the simulation world
  double maxInitialVelocity;      ///< Maximum initial velocity magnitude
  double maxInitialAcceleration;  ///< Maximum initial acceleration magnitude
  std::string
      initPositionType;  ///< How to initialize positions ("grid" or "random")

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
        initPositionType("grid") {}
};

#endif  // SRC_SIMULATION_SIMULATIONPARAMS_H_
