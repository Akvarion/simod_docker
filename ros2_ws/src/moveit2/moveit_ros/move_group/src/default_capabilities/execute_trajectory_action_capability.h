/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Kentaro Wada.
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

/*
 * Capability of execute trajectory with a ROS action.
 *
 * Author: Kentaro Wada
 * */

#pragma once

#include <moveit/move_group/move_group_capability.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>

#include <memory>

namespace move_group
{
using ExecTrajectory = moveit_msgs::action::ExecuteTrajectory;
using ExecTrajectoryGoal = rclcpp_action::ServerGoalHandle<ExecTrajectory>;

class MoveGroupExecuteTrajectoryAction : public MoveGroupCapability
{
public:
  MoveGroupExecuteTrajectoryAction();
  ~MoveGroupExecuteTrajectoryAction() override;

  void initialize() override;

private:
  void executePathCallback(const std::shared_ptr<ExecTrajectoryGoal>& goal);
  void executePath(const std::shared_ptr<ExecTrajectoryGoal>& goal, std::shared_ptr<ExecTrajectory::Result>& action_res);

  void preemptExecuteTrajectoryCallback();
  void setExecuteTrajectoryState(MoveGroupState state, const std::shared_ptr<ExecTrajectoryGoal>& goal);

  // Use our own callback group and executor to allow execution parallel to other capabilities
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;
  std::thread callback_thread_;

  std::shared_ptr<rclcpp_action::Server<ExecTrajectory>> execute_action_server_;
};

}  // namespace move_group
