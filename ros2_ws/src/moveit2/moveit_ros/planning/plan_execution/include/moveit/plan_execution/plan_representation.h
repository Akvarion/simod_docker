/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#pragma once

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <functional>

namespace plan_execution
{
struct ExecutableMotionPlan;

/** \brief Representation of a trajectory that can be executed */
struct ExecutableTrajectory
{
  ExecutableTrajectory() : trajectory_monitoring_(true)
  {
  }

  ExecutableTrajectory(const robot_trajectory::RobotTrajectoryPtr& trajectory, const std::string& description,
                       std::vector<std::string> controller_names = {})
    : trajectory_(trajectory)
    , description_(description)
    , trajectory_monitoring_(true)
    , controller_names_(std::move(controller_names))
  {
  }

  robot_trajectory::RobotTrajectoryPtr trajectory_;
  std::string description_;
  bool trajectory_monitoring_;
  collision_detection::AllowedCollisionMatrixConstPtr allowed_collision_matrix_;
  std::function<bool(const ExecutableMotionPlan*)> effect_on_success_;
  std::vector<std::string> controller_names_;
};

/// A generic representation on what a computed motion plan looks like
struct ExecutableMotionPlan
{
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_scene::PlanningSceneConstPtr planning_scene_;

  std::vector<ExecutableTrajectory> plan_components_;

  /// The trace of the trajectory recorded during execution
  robot_trajectory::RobotTrajectoryPtr executed_trajectory_;

  /// An error code reflecting what went wrong (if anything)
  moveit_msgs::msg::MoveItErrorCodes error_code_;

  planning_scene::PlanningScenePtr copyPlanningScene()
  {
    // planning_scene_ is based on the scene from this monitor
    // (either it's the monitored scene or a diff on top of it)
    // so in order to copy the scene, we must also lock the underlying monitor
    planning_scene_monitor::LockedPlanningSceneRO ls(planning_scene_monitor_);
    return planning_scene::PlanningScene::clone(planning_scene_);
  }
};

/// The signature of a function that can compute a motion plan
using ExecutableMotionPlanComputationFn = std::function<bool(ExecutableMotionPlan&)>;
}  // namespace plan_execution
