#ifndef HUMANOID_WALK__WALK__PROCESS__FOOTSTEP_PLANNER_HPP_
#define HUMANOID_WALK__WALK__PROCESS__FOOTSTEP_PLANNER_HPP_

#include <memory>
#include <vector>

#include "keisan/point3.hpp"

namespace humanoid_walk
{

class FootstepPlanner
{
public:
  enum { LEFT = 0, RIGHT = 1 };

  FootstepPlanner();

private:
  // Output trajectories
  std::vector<keisan::Point3> zmp_trajectory;
  std::vector<keisan::Point3> com_trajectory;
  std::vector<keisan::Point3> left_foot_trajectory;
  std::vector<keisan::Point3> right_foot_trajectory;

  // Robot parameters
  double com_height;
  double foot_raise_height;
  double feet_spacing;
  double foot_length;
  double foot_width;

  // Timing parameters
  double ssp_duration;
  double dsp_duration;
  double ssp_timesteps;
  size_t planned_timesteps;
};

}  // namespace humanoid_walk

#endif  // HUMANOID_WALK__WALK__PROCESS__FOOTSTEP_PLANNER_HPP_
