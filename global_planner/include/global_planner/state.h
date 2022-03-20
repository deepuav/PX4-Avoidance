#ifndef RRT_PLANNER_STATE
#define RRT_PLANNER_STATE

#include <math.h>  // abs
#include <string>
#include <tuple>

#include <geometry_msgs/Point.h>

#include "global_planner/common.h"

class State {
 public:
  State();
  State(double x, double y, double z);
  State(double x, double y);
  State(geometry_msgs::Point point);

  geometry_msgs::Point toPoint() const;

  // Get the coordinates of the center-point of the State
  double xPos() const;
  double yPos() const;
  double zPos() const;

  double manhattanDist(double _x, double _y, double _z) const;
  double distance2D(const State& b) const;
  double distance3D(const State& b) const;

};

// A GoalState has a radius and can check if a position or another state is inside
// its radius
class GoalState : public State {
 public:
  GoalState(State state, double radius = 1.0, bool is_temporary = false)
      : State(state), radius_(radius), is_temporary_(is_temporary) {}

  GoalState(double x, double y, double z, double radius = 1.0, bool is_temporary = false)
      : State(x, y, z), radius_(radius), is_temporary_(is_temporary) {}

  bool withinPlanRadius(State state) const {
    return manhattanDist(state.xPos(), state.yPos(), state.zPos()) < radius_ / 2.0;
  }

  template <typename P>
  bool withinPositionRadius(P point) const {
    return manhattanDist(point.x, point.y, point.z) < radius_;
  }

  double radius_;
  bool is_temporary_;
};

#endif  // RRT_PLANNER_STATE
