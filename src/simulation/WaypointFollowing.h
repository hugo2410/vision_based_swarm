// Copyright 2025 Hugo Birch
// Created by Hugo on 3/4/25.
//

#ifndef SRC_SIMULATION_WAYPOINTFOLLOWING_H_
#define SRC_SIMULATION_WAYPOINTFOLLOWING_H_

#include <vector>

#include "simulation/Agent.h"
#include "simulation/SimulationParams.h"

template <typename VectorType>
class WaypointFollowing {
 public:
  /**
   * @brief Constructor with optional waypoints
   * @param waypoints Custom waypoints (optional, defaults will be used if
   * empty)
   * @param params Reference to simulation parameters
   */
  WaypointFollowing(std::vector<VectorType> waypoints = {},
                    const SimulationParams& params = SimulationParams());

  /**
   * @brief Generate example accelerations for agents following waypoints.
   * @param agents Reference to the current agents
   * @return Vector of acceleration pairs for each agent
   */
  std::vector<VectorType> generateAccelerations(
      const std::vector<Agent>& agents);

  bool isWaypointReached(const std::vector<Agent>& agents,
                         const VectorType& waypoint, double reach_distance);

 private:
  std::vector<VectorType> waypoints_;
  std::vector<int>
      currentWaypointIndices_;  // Tracks each agent's current waypoint
  SimulationParams params_;     // Reference to simulation parameters

  VectorType computeSwarmCenter(const std::vector<Agent>& agents);
};

#include "WaypointFollowing.tpp"

#endif  // SRC_SIMULATION_WAYPOINTFOLLOWING_H_
