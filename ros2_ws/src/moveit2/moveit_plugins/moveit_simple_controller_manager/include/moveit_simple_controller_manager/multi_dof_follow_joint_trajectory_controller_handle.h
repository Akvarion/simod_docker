/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Unbounded Robotics Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Michael Ferguson, Ioan Sucan, E. Gil Jones */

#ifndef MULTI_DOF_FOLLOW_JOINT_CONTROLLER_HANDLE
#define MULTI_DOF_FOLLOW_JOINT_CONTROLLER_HANDLE

#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include "/ros2_ws/install/simod_moveit_action_controller/include/simod_moveit_action_controller/simod_moveit_action_controller/action/multi_dof_follow_joint_trajectory.hpp"

using MultiDofFollowJointTrajectory =
  simod_moveit_action_controller::action::MultiDofFollowJointTrajectory;

using ActionClient = rclcpp_action::Client<MultiDofFollowJointTrajectory>;

using GoalHandle = rclcpp_action::ClientGoalHandle<MultiDofFollowJointTrajectory>;

namespace moveit_simple_controller_manager
{
  const char* errorCodeToMessage(int error_code)
  {
    switch (error_code)
    {
      case control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL:
        return "SUCCESSFUL";
      case control_msgs::action::FollowJointTrajectory::Result::INVALID_GOAL:
        return "INVALID_GOAL";
      case control_msgs::action::FollowJointTrajectory::Result::INVALID_JOINTS:
        return "INVALID_JOINTS";
      case control_msgs::action::FollowJointTrajectory::Result::OLD_HEADER_TIMESTAMP:
        return "OLD_HEADER_TIMESTAMP";
      case control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED:
        return "PATH_TOLERANCE_VIOLATED";
      case control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED:
        return "GOAL_TOLERANCE_VIOLATED";
      default:
        return "unknown error";
    }
  }

  class MultiDofFollowJointTrajectoryControllerHandle : public ActionBasedControllerHandle<simod_moveit_action_controller::action::MultiDofFollowJointTrajectory>
  {

    public:
      MultiDofFollowJointTrajectoryControllerHandle(
          const rclcpp::Node::SharedPtr &node,
          const std::string &name,
          const std::string &action_ns
          ) : ActionBasedControllerHandle<simod_moveit_action_controller::action::MultiDofFollowJointTrajectory>(node,name,action_ns,"moveit.simple_controller_manager.multi_dof_follow_joint_trajectory_controller_handle"){

          }
    

      virtual bool sendTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory){
        RCLCPP_DEBUG_STREAM(LOGGER,"MultiDofFollowJointTrajectoryController: new trajectory to " << name_);

        if (!controller_action_client_)
          return false;

        if (trajectory.multi_dof_joint_trajectory.points.empty())
        {
          RCLCPP_DEBUG_STREAM(LOGGER,"MultiDofFollowJointTrajectoryController: cannot execute single-dof trajectories.");
          return false;
        }

        if (done_)
          RCLCPP_DEBUG_STREAM(LOGGER,"MultiDofFollowJointTrajectoryController: sending trajectory to " << name_);
        else
          RCLCPP_DEBUG_STREAM(LOGGER,"MultiDofFollowJointTrajectoryController: sending continuation for the currently executed trajectory to " << name_);

        MultiDofFollowJointTrajectory::Goal goal;
        typename ActionClient::SendGoalOptions goal_options;
        goal.trajectory = trajectory.multi_dof_joint_trajectory;

        goal_options.feedback_callback =
          std::bind(&MultiDofFollowJointTrajectoryControllerHandle::feedbackCallback,
                    this, std::placeholders::_1, std::placeholders::_2);

        goal_options.result_callback =
          std::bind(&MultiDofFollowJointTrajectoryControllerHandle::resultCallback,
              this, std::placeholders::_1);
        
        goal_options.goal_response_callback =
          std::bind(&MultiDofFollowJointTrajectoryControllerHandle::goalResponseCallback,
                    this, std::placeholders::_1);

        controller_action_client_->async_send_goal(goal, goal_options);

        done_ = false;
        last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
        return true;
      }

    protected:
      /**
       * @brief Check if the controller's action server is ready to receive action goals.
       * @return True if the action server is ready, false if it is not ready or does not exist.
       */
      bool isConnected() const
      {
        return controller_action_client_->action_server_is_ready();
      }

      void goalResponseCallback(
          std::shared_ptr<GoalHandle> future_handle){
        
        auto handle = future_handle.get();
        if (!handle)
          RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the action server");
        else
        {
          // controllerActiveCallback();
          RCLCPP_INFO(node_->get_logger(), "Goal accepted by the action server");
        }
      }

      void feedbackCallback(
          GoalHandle::SharedPtr,
          const std::shared_ptr<const MultiDofFollowJointTrajectory::Feedback> feedback)
      {
        //controllerFeedbackCallback(feedback);
      }

      void resultCallback(
          const ActionClient::WrappedResult & result)
      {
        controllerDoneCallback(result);
      }
      void controllerDoneCallback(
        const GoalHandle::WrappedResult& wrapped_result){
          // Output custom error message for FollowJointTrajectoryResult if necessary
          if (!wrapped_result.result)
            RCLCPP_WARN_STREAM(LOGGER, "Controller '" << name_ << "' done, no result returned");
          else if (wrapped_result.result->error_code == control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL)
            RCLCPP_INFO_STREAM(LOGGER, "Controller '" << name_ << "' successfully finished");
          else
            RCLCPP_WARN_STREAM(LOGGER, "Controller '" << name_ << "' failed with error "
                                                      << errorCodeToMessage(wrapped_result.result->error_code));
          finishControllerExecution(wrapped_result.code);
}
  };

} // end namespace moveit_simple_controller_manager

#endif // MOVEIT_PLUGINS_FOLLOW_TRAJECTORY_CONTROLLER_HANDLE