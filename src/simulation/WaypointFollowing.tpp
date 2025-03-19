#include <cmath>
#include <algorithm>

template <typename VectorType>
WaypointFollowing<VectorType>::WaypointFollowing(
    std::vector<VectorType> waypoints, const SimulationParams& params)
    : params_(params) {
  // If no waypoints are provided, use defaults
  if (waypoints.empty()) {
    if constexpr (VectorType::RowsAtCompileTime == 2) {
      waypoints_ = {
          VectorType(20.0, 20.0),  // Bottom left
          VectorType(80.0, 20.0),  // Bottom right
          VectorType(80.0, 80.0),  // Top right
          VectorType(20.0, 80.0)   // Top left
      };
    } else if constexpr (VectorType::RowsAtCompileTime == 3) {
      waypoints_ = {
          VectorType(20.0, 20.0, 0.0),  // Bottom left
          VectorType(80.0, 20.0, 0.0),  // Bottom right
          VectorType(80.0, 80.0, 0.0),  // Top right
          VectorType(20.0, 80.0, 0.0)   // Top left
      };
    }
  } else {
    waypoints_ = std::move(waypoints);
  }
}

template <typename VectorType>
std::vector<VectorType> WaypointFollowing<VectorType>::generateAccelerations(
    const std::vector<Agent>& agents) {
  std::vector<VectorType> accelerations;

  // Initialize waypoint indices if first time running
  if (currentWaypointIndices_.size() != agents.size()) {
    currentWaypointIndices_.resize(agents.size(), 0);
  }

  for (size_t i = 0; i < agents.size(); i++) {
    const auto& agent = agents[i];
    auto pos = agent.getPosition();
    auto vel = agent.getVelocity();

    // Get current waypoint index for this agent
    int& waypointIdx = currentWaypointIndices_[i];
    const auto& targetPos = waypoints_[waypointIdx];

    // Compute swarm center
    VectorType swarmCenter = computeSwarmCenter(agents);

    // Check if waypoint is reached
    if (isWaypointReached(agents, targetPos, params_.arrivalDistance)) {
      // Move to next waypoint (loop around)
      waypointIdx = (waypointIdx + 1) % waypoints_.size();
    }

    // Recalculate vector to new waypoint
    const auto& newTarget = waypoints_[waypointIdx];
    VectorType dPos = newTarget - swarmCenter;
    double distToWaypoint = dPos.norm();

    // Compute desired velocity direction
    VectorType desiredVel = VectorType::Zero();

    if (distToWaypoint > 0.001) {  // Avoid division by zero
      VectorType dir = dPos / distToWaypoint;

      // Slow down as we approach the waypoint
      double speedFactor = std::min(1.0, distToWaypoint / 10.0);
      double speed = params_.maxSpeed * speedFactor;

      // Desired velocity
      desiredVel = dir * speed;
    }

    // Compute required acceleration
    VectorType acc = (desiredVel - vel) / params_.timeStep;

    // Limit acceleration
    double accelMag = acc.norm();
    if (accelMag > params_.maxAccel) {
      acc = (acc / accelMag) * params_.maxAccel;
    }

    accelerations.push_back(acc);
  }

  return accelerations;
}

template <typename VectorType>
VectorType WaypointFollowing<VectorType>::computeSwarmCenter(
    const std::vector<Agent>& agents) {
  VectorType center = VectorType::Zero();
  if (agents.empty()) return center;

  for (const auto& agent : agents) {
    auto pos = agent.getPosition();
    center += pos;
  }
  return center / static_cast<double>(agents.size());
}

template <typename VectorType>
bool WaypointFollowing<VectorType>::isWaypointReached(
    const std::vector<Agent>& agents, const VectorType& waypoint,
    double reach_distance) {
  if (agents.empty()) return false;

  VectorType swarm_center = computeSwarmCenter(agents);
  double distance = (waypoint - swarm_center).norm();

  return distance < reach_distance;
}
