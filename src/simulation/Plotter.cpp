//
// Copyright 2025 Hugo Birch
// Created by hugo on 2/26/25.
//

#include "Plotter.h"

#include <sys/stat.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

Plotter::Plotter(const Simulation& sim, const SimulationParams& params)
    : simulation(sim), params(params) {}

void Plotter::plotTrajectories(const std::string& filename) {
  // First save trajectory data to a JSON file
  const auto& trajectories = simulation.getTrajectories();
  std::string dataFilename = "trajectory_data.json";

  std::ofstream dataFile(dataFilename);
  dataFile << "[\n";

  for (size_t i = 0; i < trajectories.size(); ++i) {
    dataFile << "  [\n";
    const auto& agentTraj = trajectories[i];

    for (size_t j = 0; j < agentTraj.size(); ++j) {
      dataFile << "    [" << agentTraj[j].x() << ", " << agentTraj[j].y()
               << "]";
      if (j < agentTraj.size() - 1) {
        dataFile << ",";
      }
      dataFile << "\n";
    }

    dataFile << "  ]";
    if (i < trajectories.size() - 1) {
      dataFile << ",";
    }
    dataFile << "\n";
  }

  dataFile << "]\n";
  dataFile.close();

  // Execute the Python script with the correct path
  std::stringstream cmd;
  cmd << "python3 " << SOURCE_DIR << "/src/simulation/plot_trajectories.py"
      << " --data_file=" << dataFilename << " --output=" << filename
      << " --world_size_x=" << params.worldSizeX
      << " --world_size_y=" << params.worldSizeY;

  std::cout << "Generating trajectory plot..." << std::endl;
  std::cout << "Executing: " << cmd.str() << std::endl;  // Debug output
  std::system(cmd.str().c_str());
  std::cout << "Plot saved to " << filename << std::endl;
}

void Plotter::createVideo(const std::string& filename, int fps) {
  // First save trajectory data to a JSON file (reusing the same code as above)
  const auto& trajectories = simulation.getTrajectories();
  std::string dataFilename = "trajectory_data.json";

  std::ofstream dataFile(dataFilename);
  dataFile << "[\n";

  for (size_t i = 0; i < trajectories.size(); ++i) {
    dataFile << "  [\n";
    const auto& agentTraj = trajectories[i];

    for (size_t j = 0; j < agentTraj.size(); ++j) {
      dataFile << "    [" << agentTraj[j].x() << ", " << agentTraj[j].y()
               << "]";
      if (j < agentTraj.size() - 1) {
        dataFile << ",";
      }
      dataFile << "\n";
    }

    dataFile << "  ]";
    if (i < trajectories.size() - 1) {
      dataFile << ",";
    }
    dataFile << "\n";
  }

  dataFile << "]\n";
  dataFile.close();

  // Execute the Python script with the correct path
  std::stringstream cmd;
  cmd << "python3 " << SOURCE_DIR << "/src/simulation/create_video.py"
      << " --data_file=" << dataFilename << " --output=" << filename
      << " --world_size_x=" << params.worldSizeX
      << " --world_size_y=" << params.worldSizeY << " --fps=" << fps;

  std::cout << "Generating simulation video..." << std::endl;
  std::cout << "Executing: " << cmd.str() << std::endl;  // Debug output
  std::system(cmd.str().c_str());
  std::cout << "Video saved to " << filename << std::endl;
}
