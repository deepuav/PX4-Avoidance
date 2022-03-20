#include "global_planner/state.h"

namespace rrt_planner {

State::State() = default;
State::State(double x, double y, double z) : State(x, y, z) {}
State::State(double x, double y) : State(x, y, 0.0) {}
State::State(geometry_msgs::Point point) : State(point.x, point.y, point.z) {}

geometry_msgs::Point State::toPoint() const {
  geometry_msgs::Point point;
  point.x = xPos();
  point.y = yPos();
  point.z = zPos();
  return point;
}

// Returns the Manhattan-distance from the center of the State
double State::manhattanDist(double _x, double _y, double _z) const {
  return std::abs(xPos() - _x) + std::abs(yPos() - _y) + std::abs(zPos() - _z);
}

// Returns the straight-line distance, disregarding the z-coordinate, to the
// center of the State
double State::distance2D(const State& b) const { return sqrt(squared(xPos() - b.xPos()) + squared(yPos() - b.yPos())); }

double State::distance3D(const State& b) const {
  return sqrt(squared(xPos() - b.xPos()) + squared(yPos() - b.yPos()) + squared(zPos() - b.zPos()));
}

}  // namespace rrt_planner
