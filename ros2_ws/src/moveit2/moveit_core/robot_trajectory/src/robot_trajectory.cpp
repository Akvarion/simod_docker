/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Adam Leeper */

#include <math.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <numeric>
#include <optional>

namespace robot_trajectory
{
RobotTrajectory::RobotTrajectory(const moveit::core::RobotModelConstPtr& robot_model)
  : robot_model_(robot_model), group_(nullptr)
{
}

RobotTrajectory::RobotTrajectory(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group)
  : robot_model_(robot_model), group_(group.empty() ? nullptr : robot_model->getJointModelGroup(group))
{
}

RobotTrajectory::RobotTrajectory(const moveit::core::RobotModelConstPtr& robot_model,
                                 const moveit::core::JointModelGroup* group)
  : robot_model_(robot_model), group_(group)
{
}

RobotTrajectory::RobotTrajectory(const RobotTrajectory& other, bool deepcopy)
{
  *this = other;  // default assignment operator performs a shallow copy
  if (deepcopy)
  {
    this->waypoints_.clear();
    for (const auto& waypoint : other.waypoints_)
    {
      this->waypoints_.emplace_back(std::make_shared<moveit::core::RobotState>(*waypoint));
    }
  }
}

const std::string& RobotTrajectory::getGroupName() const
{
  if (group_)
    return group_->getName();
  static const std::string EMPTY;
  return EMPTY;
}

double RobotTrajectory::getDuration() const
{
  return std::accumulate(duration_from_previous_.begin(), duration_from_previous_.end(), 0.0);
}

double RobotTrajectory::getAverageSegmentDuration() const
{
  if (duration_from_previous_.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("RobotTrajectory"), "Too few waypoints to calculate a duration. Returning 0.");
    return 0.0;
  }

  // If the initial segment has a duration of 0, exclude it from the average calculation
  if (duration_from_previous_[0] == 0)
  {
    if (duration_from_previous_.size() <= 1)
    {
      RCLCPP_WARN(rclcpp::get_logger("RobotTrajectory"), "First and only waypoint has a duration of 0.");
      return 0.0;
    }
    else
      return getDuration() / static_cast<double>(duration_from_previous_.size() - 1);
  }
  else
    return getDuration() / static_cast<double>(duration_from_previous_.size());
}

void RobotTrajectory::swap(RobotTrajectory& other)
{
  robot_model_.swap(other.robot_model_);
  std::swap(group_, other.group_);
  waypoints_.swap(other.waypoints_);
  duration_from_previous_.swap(other.duration_from_previous_);
}

RobotTrajectory& RobotTrajectory::append(const RobotTrajectory& source, double dt, size_t start_index, size_t end_index)
{
  end_index = std::min(end_index, source.waypoints_.size());
  if (start_index >= end_index)
    return *this;
  waypoints_.insert(waypoints_.end(), std::next(source.waypoints_.begin(), start_index),
                    std::next(source.waypoints_.begin(), end_index));
  std::size_t index = duration_from_previous_.size();
  duration_from_previous_.insert(duration_from_previous_.end(),
                                 std::next(source.duration_from_previous_.begin(), start_index),
                                 std::next(source.duration_from_previous_.begin(), end_index));
  if (duration_from_previous_.size() > index)
    duration_from_previous_[index] = dt;

  return *this;
}

RobotTrajectory& RobotTrajectory::reverse()
{
  std::reverse(waypoints_.begin(), waypoints_.end());
  for (moveit::core::RobotStatePtr& waypoint : waypoints_)
  {
    // reversing the trajectory implies inverting the velocity profile
    waypoint->invertVelocity();
  }
  if (!duration_from_previous_.empty())
  {
    duration_from_previous_.push_back(duration_from_previous_.front());
    std::reverse(duration_from_previous_.begin(), duration_from_previous_.end());
    duration_from_previous_.pop_back();
  }

  return *this;
}

RobotTrajectory& RobotTrajectory::unwind()
{
  if (waypoints_.empty())
    return *this;

  const std::vector<const moveit::core::JointModel*>& cont_joints =
      group_ ? group_->getContinuousJointModels() : robot_model_->getContinuousJointModels();

  for (const moveit::core::JointModel* cont_joint : cont_joints)
  {
    // unwrap continuous joints
    double running_offset = 0.0;
    double last_value = waypoints_[0]->getJointPositions(cont_joint)[0];

    for (std::size_t j = 1; j < waypoints_.size(); ++j)
    {
      double current_value = waypoints_[j]->getJointPositions(cont_joint)[0];
      if (last_value > current_value + M_PI)
        running_offset += 2.0 * M_PI;
      else if (current_value > last_value + M_PI)
        running_offset -= 2.0 * M_PI;

      last_value = current_value;
      if (running_offset > std::numeric_limits<double>::epsilon() ||
          running_offset < -std::numeric_limits<double>::epsilon())
      {
        current_value += running_offset;
        waypoints_[j]->setJointPositions(cont_joint, &current_value);
      }
    }
  }
  for (moveit::core::RobotStatePtr& waypoint : waypoints_)
    waypoint->update();

  return *this;
}

RobotTrajectory& RobotTrajectory::unwind(const moveit::core::RobotState& state)
{
  if (waypoints_.empty())
    return *this;

  const std::vector<const moveit::core::JointModel*>& cont_joints =
      group_ ? group_->getContinuousJointModels() : robot_model_->getContinuousJointModels();

  for (const moveit::core::JointModel* cont_joint : cont_joints)
  {
    double reference_value0 = state.getJointPositions(cont_joint)[0];
    double reference_value = reference_value0;
    cont_joint->enforcePositionBounds(&reference_value);

    // unwrap continuous joints
    double running_offset = reference_value0 - reference_value;

    double last_value = waypoints_[0]->getJointPositions(cont_joint)[0];
    if (running_offset > std::numeric_limits<double>::epsilon() ||
        running_offset < -std::numeric_limits<double>::epsilon())
    {
      double current_value = last_value + running_offset;
      waypoints_[0]->setJointPositions(cont_joint, &current_value);
    }

    for (std::size_t j = 1; j < waypoints_.size(); ++j)
    {
      double current_value = waypoints_[j]->getJointPositions(cont_joint)[0];
      if (last_value > current_value + M_PI)
        running_offset += 2.0 * M_PI;
      else if (current_value > last_value + M_PI)
        running_offset -= 2.0 * M_PI;

      last_value = current_value;
      if (running_offset > std::numeric_limits<double>::epsilon() ||
          running_offset < -std::numeric_limits<double>::epsilon())
      {
        current_value += running_offset;
        waypoints_[j]->setJointPositions(cont_joint, &current_value);
      }
    }
  }
  for (moveit::core::RobotStatePtr& waypoint : waypoints_)
    waypoint->update();

  return *this;
}

void RobotTrajectory::getRobotTrajectoryMsg(moveit_msgs::msg::RobotTrajectory& trajectory,
                                            const std::vector<std::string>& joint_filter) const
{
  trajectory = moveit_msgs::msg::RobotTrajectory();
  if (waypoints_.empty())
    return;
  const std::vector<const moveit::core::JointModel*>& jnts =
      group_ ? group_->getActiveJointModels() : robot_model_->getActiveJointModels();

  std::vector<const moveit::core::JointModel*> onedof;
  std::vector<const moveit::core::JointModel*> mdof;
  trajectory.joint_trajectory.joint_names.clear();
  trajectory.multi_dof_joint_trajectory.joint_names.clear();

  for (const moveit::core::JointModel* active_joint : jnts)
  {
    // only consider joints listed in joint_filter
    if (!joint_filter.empty() &&
        std::find(joint_filter.begin(), joint_filter.end(), active_joint->getName()) == joint_filter.end())
      continue;

    if (active_joint->getVariableCount() == 1)
    {
      trajectory.joint_trajectory.joint_names.push_back(active_joint->getName());
      onedof.push_back(active_joint);
    }
    else
    {
      trajectory.multi_dof_joint_trajectory.joint_names.push_back(active_joint->getName());
      mdof.push_back(active_joint);
    }
  }

  if (!onedof.empty())
  {
    trajectory.joint_trajectory.header.frame_id = robot_model_->getModelFrame();
    trajectory.joint_trajectory.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    trajectory.joint_trajectory.points.resize(waypoints_.size());
  }

  if (!mdof.empty())
  {
    trajectory.multi_dof_joint_trajectory.header.frame_id = robot_model_->getModelFrame();
    trajectory.multi_dof_joint_trajectory.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    trajectory.multi_dof_joint_trajectory.points.resize(waypoints_.size());
  }

  static const auto ZERO_DURATION = rclcpp::Duration::from_seconds(0);
  double total_time = 0.0;
  for (std::size_t i = 0; i < waypoints_.size(); ++i)
  {
    if (duration_from_previous_.size() > i)
      total_time += duration_from_previous_[i];

    if (!onedof.empty())
    {
      trajectory.joint_trajectory.points[i].positions.resize(onedof.size());
      trajectory.joint_trajectory.points[i].velocities.reserve(onedof.size());

      for (std::size_t j = 0; j < onedof.size(); ++j)
      {
        trajectory.joint_trajectory.points[i].positions[j] =
            waypoints_[i]->getVariablePosition(onedof[j]->getFirstVariableIndex());
        // if we have velocities/accelerations/effort, copy those too
        if (waypoints_[i]->hasVelocities())
          trajectory.joint_trajectory.points[i].velocities.push_back(
              waypoints_[i]->getVariableVelocity(onedof[j]->getFirstVariableIndex()));
        if (waypoints_[i]->hasAccelerations())
          trajectory.joint_trajectory.points[i].accelerations.push_back(
              waypoints_[i]->getVariableAcceleration(onedof[j]->getFirstVariableIndex()));
        if (waypoints_[i]->hasEffort())
          trajectory.joint_trajectory.points[i].effort.push_back(
              waypoints_[i]->getVariableEffort(onedof[j]->getFirstVariableIndex()));
      }
      // clear velocities if we have an incomplete specification
      if (trajectory.joint_trajectory.points[i].velocities.size() != onedof.size())
        trajectory.joint_trajectory.points[i].velocities.clear();
      // clear accelerations if we have an incomplete specification
      if (trajectory.joint_trajectory.points[i].accelerations.size() != onedof.size())
        trajectory.joint_trajectory.points[i].accelerations.clear();
      // clear effort if we have an incomplete specification
      if (trajectory.joint_trajectory.points[i].effort.size() != onedof.size())
        trajectory.joint_trajectory.points[i].effort.clear();

      if (duration_from_previous_.size() > i)
        trajectory.joint_trajectory.points[i].time_from_start = rclcpp::Duration::from_seconds(total_time);
      else
        trajectory.joint_trajectory.points[i].time_from_start = ZERO_DURATION;
    }
    if (!mdof.empty())
    {
      trajectory.multi_dof_joint_trajectory.points[i].transforms.resize(mdof.size());
      for (std::size_t j = 0; j < mdof.size(); ++j)
      {
        geometry_msgs::msg::TransformStamped ts = tf2::eigenToTransform(waypoints_[i]->getJointTransform(mdof[j]));
        trajectory.multi_dof_joint_trajectory.points[i].transforms[j] = ts.transform;
        // TODO: currently only checking for planar multi DOF joints / need to add check for floating
        if (waypoints_[i]->hasVelocities() && (mdof[j]->getType() == moveit::core::JointModel::JointType::PLANAR))
        {
          const std::vector<std::string> names = mdof[j]->getVariableNames();
          const double* velocities = waypoints_[i]->getJointVelocities(mdof[j]);

          geometry_msgs::msg::Twist point_velocity;

          for (std::size_t k = 0; k < names.size(); ++k)
          {
            if (names[k].find("/x") != std::string::npos)
            {
              point_velocity.linear.x = velocities[k];
            }
            else if (names[k].find("/y") != std::string::npos)
            {
              point_velocity.linear.y = velocities[k];
            }
            else if (names[k].find("/z") != std::string::npos)
            {
              point_velocity.linear.z = velocities[k];
            }
            else if (names[k].find("/theta") != std::string::npos)
            {
              point_velocity.angular.z = velocities[k];
            }
          }
          trajectory.multi_dof_joint_trajectory.points[i].velocities.push_back(point_velocity);
        }
      }
      if (duration_from_previous_.size() > i)
        trajectory.multi_dof_joint_trajectory.points[i].time_from_start = rclcpp::Duration::from_seconds(total_time);
      else
        trajectory.multi_dof_joint_trajectory.points[i].time_from_start = ZERO_DURATION;
    }
  }
}

RobotTrajectory& RobotTrajectory::setRobotTrajectoryMsg(const moveit::core::RobotState& reference_state,
                                                        const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  // make a copy just in case the next clear() removes the memory for the reference passed in
  const moveit::core::RobotState copy(reference_state);  // NOLINT(performance-unnecessary-copy-initialization)
  clear();
  std::size_t state_count = trajectory.points.size();
  rclcpp::Time last_time_stamp = trajectory.header.stamp;
  rclcpp::Time this_time_stamp = last_time_stamp;

  for (std::size_t i = 0; i < state_count; ++i)
  {
    this_time_stamp = rclcpp::Time(trajectory.header.stamp) + trajectory.points[i].time_from_start;
    auto st = std::make_shared<moveit::core::RobotState>(copy);
    st->setVariablePositions(trajectory.joint_names, trajectory.points[i].positions);
    if (!trajectory.points[i].velocities.empty())
      st->setVariableVelocities(trajectory.joint_names, trajectory.points[i].velocities);
    if (!trajectory.points[i].accelerations.empty())
      st->setVariableAccelerations(trajectory.joint_names, trajectory.points[i].accelerations);
    if (!trajectory.points[i].effort.empty())
      st->setVariableEffort(trajectory.joint_names, trajectory.points[i].effort);
    addSuffixWayPoint(st, (this_time_stamp - last_time_stamp).seconds());
    last_time_stamp = this_time_stamp;
  }

  return *this;
}

RobotTrajectory& RobotTrajectory::setRobotTrajectoryMsg(const moveit::core::RobotState& reference_state,
                                                        const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  // make a copy just in case the next clear() removes the memory for the reference passed in
  const moveit::core::RobotState& copy = reference_state;
  clear();

  std::size_t state_count =
      std::max(trajectory.joint_trajectory.points.size(), trajectory.multi_dof_joint_trajectory.points.size());
  rclcpp::Time last_time_stamp = trajectory.joint_trajectory.points.empty() ?
                                     trajectory.multi_dof_joint_trajectory.header.stamp :
                                     trajectory.joint_trajectory.header.stamp;
  rclcpp::Time this_time_stamp = last_time_stamp;

  for (std::size_t i = 0; i < state_count; ++i)
  {
    auto st = std::make_shared<moveit::core::RobotState>(copy);
    if (trajectory.joint_trajectory.points.size() > i)
    {
      st->setVariablePositions(trajectory.joint_trajectory.joint_names, trajectory.joint_trajectory.points[i].positions);
      if (!trajectory.joint_trajectory.points[i].velocities.empty())
        st->setVariableVelocities(trajectory.joint_trajectory.joint_names,
                                  trajectory.joint_trajectory.points[i].velocities);
      if (!trajectory.joint_trajectory.points[i].accelerations.empty())
        st->setVariableAccelerations(trajectory.joint_trajectory.joint_names,
                                     trajectory.joint_trajectory.points[i].accelerations);
      if (!trajectory.joint_trajectory.points[i].effort.empty())
        st->setVariableEffort(trajectory.joint_trajectory.joint_names, trajectory.joint_trajectory.points[i].effort);
      this_time_stamp = rclcpp::Time(trajectory.joint_trajectory.header.stamp) +
                        trajectory.joint_trajectory.points[i].time_from_start;
    }
    if (trajectory.multi_dof_joint_trajectory.points.size() > i)
    {
      for (std::size_t j = 0; j < trajectory.multi_dof_joint_trajectory.joint_names.size(); ++j)
      {
        Eigen::Isometry3d t = tf2::transformToEigen(trajectory.multi_dof_joint_trajectory.points[i].transforms[j]);
        st->setJointPositions(trajectory.multi_dof_joint_trajectory.joint_names[j], t);
      }
      this_time_stamp = rclcpp::Time(trajectory.multi_dof_joint_trajectory.header.stamp) +
                        trajectory.multi_dof_joint_trajectory.points[i].time_from_start;
    }

    addSuffixWayPoint(st, (this_time_stamp - last_time_stamp).seconds());
    last_time_stamp = this_time_stamp;
  }
  return *this;
}

RobotTrajectory& RobotTrajectory::setRobotTrajectoryMsg(const moveit::core::RobotState& reference_state,
                                                        const moveit_msgs::msg::RobotState& state,
                                                        const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  moveit::core::RobotState st(reference_state);
  moveit::core::robotStateMsgToRobotState(state, st);
  return setRobotTrajectoryMsg(st, trajectory);
}

void RobotTrajectory::findWayPointIndicesForDurationAfterStart(const double& duration, int& before, int& after,
                                                               double& blend) const
{
  if (duration < 0.0)
  {
    before = 0;
    after = 0;
    blend = 0;
    return;
  }

  // Find indices
  std::size_t index = 0, num_points = waypoints_.size();
  double running_duration = 0.0;
  for (; index < num_points; ++index)
  {
    running_duration += duration_from_previous_[index];
    if (running_duration >= duration)
      break;
  }
  before = std::max<int>(index - 1, 0);
  after = std::min<int>(index, num_points - 1);

  // Compute duration blend
  double before_time = running_duration - duration_from_previous_[index];
  if (after == before)
    blend = 1.0;
  else
    blend = (duration - before_time) / duration_from_previous_[index];
}

double RobotTrajectory::getWayPointDurationFromStart(std::size_t index) const
{
  if (duration_from_previous_.empty())
    return 0.0;
  if (index >= duration_from_previous_.size())
    index = duration_from_previous_.size() - 1;

  double time = 0.0;
  for (std::size_t i = 0; i <= index; ++i)
    time += duration_from_previous_[i];
  return time;
}

double RobotTrajectory::getWaypointDurationFromStart(std::size_t index) const
{
  return getWayPointDurationFromStart(index);
}

bool RobotTrajectory::getStateAtDurationFromStart(const double request_duration,
                                                  moveit::core::RobotStatePtr& output_state) const
{
  // If there are no waypoints we can't do anything
  if (getWayPointCount() == 0)
    return false;

  int before = 0, after = 0;
  double blend = 1.0;
  findWayPointIndicesForDurationAfterStart(request_duration, before, after, blend);
  // ROS_DEBUG_NAMED("robot_trajectory", "Interpolating %.3f of the way between index %d and %d.", blend, before,
  // after);
  waypoints_[before]->interpolate(*waypoints_[after], blend, *output_state);
  return true;
}

void RobotTrajectory::print(std::ostream& out, std::vector<int> variable_indexes) const
{
  size_t num_points = getWayPointCount();
  if (num_points == 0)
  {
    out << "Empty trajectory.";
    return;
  }

  std::ios::fmtflags old_settings = out.flags();
  int old_precision = out.precision();
  out << std::fixed << std::setprecision(3);

  out << "Trajectory has " << num_points << " points over " << getDuration() << " seconds\n";

  if (variable_indexes.empty())
  {
    if (group_)
    {
      variable_indexes = group_->getVariableIndexList();
    }
    else
    {
      // use all variables
      variable_indexes.resize(robot_model_->getVariableCount());
      std::iota(variable_indexes.begin(), variable_indexes.end(), 0);
    }
  }

  for (size_t p_i = 0; p_i < num_points; ++p_i)
  {
    const moveit::core::RobotState& point = getWayPoint(p_i);
    out << "  waypoint " << std::setw(3) << p_i;
    out << " time " << std::setw(5) << getWayPointDurationFromStart(p_i);
    out << " pos ";
    for (int index : variable_indexes)
    {
      out << std::setw(6) << point.getVariablePosition(index) << " ";
    }
    if (point.hasVelocities())
    {
      out << "vel ";
      for (int index : variable_indexes)
      {
        out << std::setw(6) << point.getVariableVelocity(index) << " ";
      }
    }
    if (point.hasAccelerations())
    {
      out << "acc ";
      for (int index : variable_indexes)
      {
        out << std::setw(6) << point.getVariableAcceleration(index) << " ";
      }
    }
    if (point.hasEffort())
    {
      out << "eff ";
      for (int index : variable_indexes)
      {
        out << std::setw(6) << point.getVariableEffort(index) << " ";
      }
    }
    out << "\n";
  }

  out.flags(old_settings);
  out.precision(old_precision);
  out.flush();
}

std::ostream& operator<<(std::ostream& out, const RobotTrajectory& trajectory)
{
  trajectory.print(out);
  return out;
}

double path_length(RobotTrajectory const& trajectory)
{
  auto trajectory_length = 0.0;
  for (std::size_t index = 1; index < trajectory.getWayPointCount(); ++index)
  {
    auto const& first = trajectory.getWayPoint(index - 1);
    auto const& second = trajectory.getWayPoint(index);
    trajectory_length += first.distance(second);
  }
  return trajectory_length;
}

std::optional<double> smoothness(RobotTrajectory const& trajectory)
{
  if (trajectory.getWayPointCount() > 2)
  {
    auto smoothness = 0.0;
    double a = trajectory.getWayPoint(0).distance(trajectory.getWayPoint(1));
    for (std::size_t k = 2; k < trajectory.getWayPointCount(); ++k)
    {
      // view the path as a sequence of segments, and look at the triangles it forms:
      //          s1
      //          /\          s4
      //      a  /  \ b       |
      //        /    \        |
      //       /......\_______|
      //     s0    c   s2     s3

      // use Pythagoras generalized theorem to find the cos of the angle between segments a and b
      double b = trajectory.getWayPoint(k - 1).distance(trajectory.getWayPoint(k));
      double cdist = trajectory.getWayPoint(k - 2).distance(trajectory.getWayPoint(k));
      double acos_value = (a * a + b * b - cdist * cdist) / (2.0 * a * b);
      if (acos_value > -1.0 && acos_value < 1.0)
      {
        // the smoothness is actually the outside angle of the one we compute
        double angle = (M_PI - acos(acos_value));

        // and we normalize by the length of the segments
        double u = 2.0 * angle;  /// (a + b);
        smoothness += u * u;
      }
      a = b;
    }
    smoothness /= (double)trajectory.getWayPointCount();
    return smoothness;
  }
  // In case the path is to short, no value is returned
  return std::nullopt;
}

std::optional<double> waypoint_density(RobotTrajectory const& trajectory)
{
  // Only calculate density if more than one waypoint exists
  if (trajectory.getWayPointCount() > 1)
  {
    // Calculate path length
    auto const length = path_length(trajectory);
    if (length > 0.0)
    {
      auto density = (double)trajectory.getWayPointCount() / length;
      return density;
    }
  }
  // Trajectory is empty, a single point or path length is zero
  return std::nullopt;
}

}  // end of namespace robot_trajectory
