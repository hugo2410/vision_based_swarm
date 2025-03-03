//
// Copyright 2025 Hugo Birch
// Created by hugo on 2/26/25.
//
/**
 * @file Plotter.h
 * @brief Defines the Plotter class for visualizing simulation results
 * @author Hugo Birch
 * @date 2025-02-26
 */

#ifndef SRC_SIMULATION_PLOTTER_H_
#define SRC_SIMULATION_PLOTTER_H_

#include <string>

#include "Simulation.h"
#include "SimulationParams.h"

/**
 * @class Plotter
 * @brief Provides visualization tools for simulation data
 *
 * The Plotter class generates static plots and videos from simulation
 * trajectory data using Python scripts with matplotlib.
 */
class Plotter {
 public:
  /**
   * @brief Constructor
   * @param sim Reference to the simulation containing trajectory data
   * @param params Reference to the simulation parameters
   */
  Plotter(const Simulation& sim, const SimulationParams& params);

  /**
   * @brief Generate a static plot showing agent trajectories
   * @param filename The output image filename
   */
  void plotTrajectories(const std::string& filename = "trajectories.png");

  /**
   * @brief Generate a video animation of the simulation
   * @param filename The output video filename
   * @param fps Frames per second for the video
   */
  void createVideo(const std::string& filename = "simulation.mp4",
                   int fps = 30);

 private:
  const Simulation& simulation;    ///< Reference to the simulation
  const SimulationParams& params;  ///< Reference to the simulation parameters
};

#endif  // SRC_SIMULATION_PLOTTER_H_
