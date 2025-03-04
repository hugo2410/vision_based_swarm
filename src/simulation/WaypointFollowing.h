// Copyright 2025 Hugo Birch
// Created by Hugo on 3/4/25.
//

#ifndef SRC_SIMULATION_WAYPOINTFOLLOWING_H_
#define SRC_SIMULATION_WAYPOINTFOLLOWING_H_

#include <vector>

#include "simulation/Agent.h"

class WaypointFollowing {
 public:
  /**
   * @brief Constructor with optional waypoints
   * @param waypoints Custom waypoints (optional, defaults will be used if
   * empty)
   */
  explicit WaypointFollowing(
      std::vector<std::pair<double, double>> waypoints = {});

  /**
   * @brief Generate example accelerations for agents following waypoints.
   * @param agents Reference to the current agents
   * @return Vector of acceleration pairs for each agent
   */
  std::vector<std::pair<double, double>> generateAccelerations(
      const std::vector<Agent>& agents);

 private:
  std::vector<std::pair<double, double>> waypoints_;
  std::vector<int>
      currentWaypointIndices_;  // Tracks each agent's current waypoint
};

#endif  // SRC_SIMULATION_WAYPOINTFOLLOWING_H_
