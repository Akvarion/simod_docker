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

#include <moveit/plan_execution/plan_execution.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/utils/message_checks.h>
#include <boost/algorithm/string/join.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

// #include <dynamic_reconfigure/server.h>
// #include <moveit_ros_planning/PlanExecutionDynamicReconfigureConfig.h>

namespace plan_execution
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros.plan_execution");

// class PlanExecution::DynamicReconfigureImpl
// {
// public:
//   DynamicReconfigureImpl(PlanExecution* owner)
//     : owner_(owner)  //, dynamic_reconfigure_server_(ros::NodeHandle("~/plan_execution"))
//   {
//     // dynamic_reconfigure_server_.setCallback(
//     //     [this](const auto& config, uint32_t level) { dynamicReconfigureCallback(config, level); });
//   }
//
// private:
//   // void dynamicReconfigureCallback(const PlanExecutionDynamicReconfigureConfig& config, uint32_t level)
//   // {
//   //   owner_->setMaxReplanAttempts(config.max_replan_attempts);
//   //   owner_->setTrajectoryStateRecordingFrequency(config.record_trajectory_state_frequency);
//   // }
//
//   PlanExecution* owner_;
//   // dynamic_reconfigure::Server<PlanExecutionDynamicReconfigureConfig> dynamic_reconfigure_server_;
// };
}  // namespace plan_execution

plan_execution::PlanExecution::PlanExecution(
    const rclcpp::Node::SharedPtr& node, const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
    const trajectory_execution_manager::TrajectoryExecutionManagerPtr& trajectory_execution)
  : node_(node), planning_scene_monitor_(planning_scene_monitor), trajectory_execution_manager_(trajectory_execution)
{
  if (!trajectory_execution_manager_)
    trajectory_execution_manager_ = std::make_shared<trajectory_execution_manager::TrajectoryExecutionManager>(
        node_, planning_scene_monitor_->getRobotModel(), planning_scene_monitor_->getStateMonitor());

  default_max_replan_attempts_ = 5;

  new_scene_update_ = false;

  // we want to be notified when new information is available
  planning_scene_monitor_->addUpdateCallback(
      [this](const planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type) {
        planningSceneUpdatedCallback(update_type);
      });

  // start the dynamic-reconfigure server
  // reconfigure_impl_ = new DynamicReconfigureImpl(this);
}

plan_execution::PlanExecution::~PlanExecution()
{
  // delete reconfigure_impl_;
}

void plan_execution::PlanExecution::stop()
{
  preempt_.request();
}

std::string plan_execution::PlanExecution::getErrorCodeString(const moveit_msgs::msg::MoveItErrorCodes& error_code)
{
  if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    return "Success";
  else if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME)
    return "Invalid group name";
  else if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED)
    return "Planning failed.";
  else if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN)
    return "Invalid motion plan";
  else if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA)
    return "Unable to acquire sensor data";
  else if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE)
    return "Motion plan invalidated by environment change";
  else if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED)
    return "Controller failed during execution";
  else if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT)
    return "Timeout reached";
  else if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::PREEMPTED)
    return "Preempted";
  else if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS)
    return "Invalid goal constraints";
  else if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::INVALID_OBJECT_NAME)
    return "Invalid object name";
  else if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::FAILURE)
    return "Catastrophic failure";
  return "Unknown event";
}

void plan_execution::PlanExecution::planAndExecute(ExecutableMotionPlan& plan, const Options& opt)
{
  plan.planning_scene_monitor_ = planning_scene_monitor_;
  plan.planning_scene_ = planning_scene_monitor_->getPlanningScene();
  planAndExecuteHelper(plan, opt);
}

void plan_execution::PlanExecution::planAndExecute(ExecutableMotionPlan& plan,
                                                   const moveit_msgs::msg::PlanningScene& scene_diff,
                                                   const Options& opt)
{
  if (moveit::core::isEmpty(scene_diff))
    planAndExecute(plan, opt);
  else
  {
    plan.planning_scene_monitor_ = planning_scene_monitor_;
    {
      planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);  // lock the scene so that it does
                                                                                      // not modify the world
                                                                                      // representation while diff() is
                                                                                      // called
      plan.planning_scene_ = lscene->diff(scene_diff);
    }
    planAndExecuteHelper(plan, opt);
  }
}

void plan_execution::PlanExecution::planAndExecuteHelper(ExecutableMotionPlan& plan, const Options& opt)
{
  // perform initial configuration steps & various checks
  preempt_.checkAndClear();  // clear any previous preempt_ request

  bool preempt_requested = false;

  // run the actual motion plan & execution
  unsigned int max_replan_attempts =
      opt.replan_ ? (opt.replan_attempts_ > 0 ? opt.replan_attempts_ : default_max_replan_attempts_) : 1;
  unsigned int replan_attempts = 0;
  bool previously_solved = false;

  // run a planning loop for at most the maximum replanning attempts;
  // re-planning is executed only in case of known types of failures (e.g., environment changed)
  do
  {
    replan_attempts++;
    RCLCPP_INFO(LOGGER, "Planning attempt %u of at most %u", replan_attempts, max_replan_attempts);

    if (opt.before_plan_callback_)
      opt.before_plan_callback_();

    new_scene_update_ = false;  // we clear any scene updates to be evaluated because we are about to compute a new
                                // plan, which should consider most recent updates already

    // if we never had a solved plan, or there is no specified way of fixing plans, just call the planner; otherwise,
    // try to repair the plan we previously had;
    bool solved =
        (!previously_solved || !opt.repair_plan_callback_) ?
            opt.plan_callback_(plan) :
            opt.repair_plan_callback_(plan, trajectory_execution_manager_->getCurrentExpectedTrajectoryIndex());

    preempt_requested = preempt_.checkAndClear();
    if (preempt_requested)
      break;

    // if planning fails in a manner that is not recoverable, we exit the loop,
    // otherwise, we attempt to continue, if replanning attempts are left
    if (plan.error_code_.val == moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED ||
        plan.error_code_.val == moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN ||
        plan.error_code_.val == moveit_msgs::msg::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA)
    {
      if (plan.error_code_.val == moveit_msgs::msg::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA &&
          opt.replan_delay_ > 0.0)
      {
        auto replan_delay_seconds = std::chrono::duration<double>(opt.replan_delay_);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(replan_delay_seconds));
      }
      continue;
    }

    // abort if no plan was found
    if (solved)
      previously_solved = true;
    else
      break;

    if (plan.error_code_.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      if (opt.before_execution_callback_)
        opt.before_execution_callback_();

      preempt_requested = preempt_.checkAndClear();
      if (preempt_requested)
        break;

      // execute the trajectory, and monitor its execution
      plan.error_code_ = executeAndMonitor(plan, false);
    }

    if (plan.error_code_.val == moveit_msgs::msg::MoveItErrorCodes::PREEMPTED)
      preempt_requested = true;

    // if execution succeeded or failed in a manner that we do not consider recoverable, we exit the loop (with failure)
    if (plan.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE)
      break;
    else
    {
      // otherwise, we wait (if needed)
      if (opt.replan_delay_ > 0.0)
      {
        RCLCPP_INFO(LOGGER, "Waiting for a %lf seconds before attempting a new plan ...", opt.replan_delay_);
        auto replan_delay_seconds = std::chrono::duration<double>(opt.replan_delay_);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(replan_delay_seconds));
        RCLCPP_INFO(node_->get_logger(), "Done waiting");
      }
    }

    preempt_requested = preempt_.checkAndClear();
    if (preempt_requested)
      break;

  } while (replan_attempts < max_replan_attempts);

  if (preempt_requested)
  {
    RCLCPP_DEBUG(LOGGER, "PlanExecution was preempted");
    plan.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
  }

  if (opt.done_callback_)
    opt.done_callback_();

  if (plan.error_code_.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_DEBUG(LOGGER, "PlanExecution finished successfully.");
  }
  else
  {
    RCLCPP_DEBUG(LOGGER, "PlanExecution terminating with error code %d - '%s'", plan.error_code_.val,
                 getErrorCodeString(plan.error_code_).c_str());
  }
}

bool plan_execution::PlanExecution::isRemainingPathValid(const ExecutableMotionPlan& plan,
                                                         const std::pair<int, int>& path_segment)
{
  if (path_segment.first >= 0 &&
      plan.plan_components_[path_segment.first].trajectory_monitoring_)  // If path_segment.second <= 0, the function
                                                                         // will fallback to check the entire trajectory
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);  // lock the scene so that it
                                                                                         // does not modify the world
                                                                                         // representation while
                                                                                         // isStateValid() is called
    const robot_trajectory::RobotTrajectory& t = *plan.plan_components_[path_segment.first].trajectory_;
    const collision_detection::AllowedCollisionMatrix* acm =
        plan.plan_components_[path_segment.first].allowed_collision_matrix_.get();
    std::size_t wpc = t.getWayPointCount();
    collision_detection::CollisionRequest req;
    req.group_name = t.getGroupName();
    for (std::size_t i = std::max(path_segment.second - 1, 0); i < wpc; ++i)
    {
      collision_detection::CollisionResult res;
      if (acm)
        plan.planning_scene_->checkCollisionUnpadded(req, res, t.getWayPoint(i), *acm);
      else
        plan.planning_scene_->checkCollisionUnpadded(req, res, t.getWayPoint(i));

      if (res.collision || !plan.planning_scene_->isStateFeasible(t.getWayPoint(i), false))
      {
        RCLCPP_INFO(LOGGER, "Trajectory component '%s' is invalid for waypoint %ld out of %ld",
                    plan.plan_components_[path_segment.first].description_.c_str(), i, wpc);

        // call the same functions again, in verbose mode, to show what issues have been detected
        plan.planning_scene_->isStateFeasible(t.getWayPoint(i), true);
        req.verbose = true;
        res.clear();
        if (acm)
          plan.planning_scene_->checkCollisionUnpadded(req, res, t.getWayPoint(i), *acm);
        else
          plan.planning_scene_->checkCollisionUnpadded(req, res, t.getWayPoint(i));
        return false;
      }
    }
  }
  return true;
}

moveit_msgs::msg::MoveItErrorCodes plan_execution::PlanExecution::executeAndMonitor(ExecutableMotionPlan& plan,
                                                                                    bool reset_preempted)
{
  if (reset_preempted)
    preempt_.checkAndClear();

  if (!plan.planning_scene_monitor_)
    plan.planning_scene_monitor_ = planning_scene_monitor_;
  if (!plan.planning_scene_)
    plan.planning_scene_ = planning_scene_monitor_->getPlanningScene();

  moveit_msgs::msg::MoveItErrorCodes result;

  // try to execute the trajectory
  execution_complete_ = true;

  if (!trajectory_execution_manager_)
  {
    RCLCPP_ERROR(LOGGER, "No trajectory execution manager");
    result.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
    return result;
  }

  if (plan.plan_components_.empty())
  {
    result.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return result;
  }

  execution_complete_ = false;

  // push the trajectories we have slated for execution to the trajectory execution manager
  int prev = -1;
  for (std::size_t i = 0; i < plan.plan_components_.size(); ++i)
  {
    // \todo should this be in trajectory_execution ? Maybe. Then that will have to use kinematic_trajectory too;
    // splitting trajectories for controllers becomes interesting: tied to groups instead of joints. this could cause
    // some problems
    // in the meantime we do a hack:

    bool unwound = false;
    for (std::size_t j = 0; j < i; ++j)
      // if we ran unwind on a path for the same group
      if (plan.plan_components_[j].trajectory_ &&
          plan.plan_components_[j].trajectory_->getGroup() == plan.plan_components_[i].trajectory_->getGroup() &&
          !plan.plan_components_[j].trajectory_->empty())
      {
        plan.plan_components_[i].trajectory_->unwind(plan.plan_components_[j].trajectory_->getLastWayPoint());
        unwound = true;
        break;
      }

    if (!unwound)
    {
      // unwind the path to execute based on the current state of the system
      if (prev < 0)
        plan.plan_components_[i].trajectory_->unwind(
            plan.planning_scene_monitor_ && plan.planning_scene_monitor_->getStateMonitor() ?
                *plan.planning_scene_monitor_->getStateMonitor()->getCurrentState() :
                plan.planning_scene_->getCurrentState());
      else
        plan.plan_components_[i].trajectory_->unwind(plan.plan_components_[prev].trajectory_->getLastWayPoint());
    }

    if (plan.plan_components_[i].trajectory_ && !plan.plan_components_[i].trajectory_->empty())
      prev = i;

    // convert to message, pass along
    moveit_msgs::msg::RobotTrajectory msg;
    plan.plan_components_[i].trajectory_->getRobotTrajectoryMsg(msg);
    if (!trajectory_execution_manager_->push(msg, plan.plan_components_[i].controller_names_))
    {
      RCLCPP_ERROR(LOGGER, "Apparently trajectory initialization failed");
      execution_complete_ = true;
      result.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
      return result;
    }
  }

  if (!trajectory_monitor_ && planning_scene_monitor_->getStateMonitor())
  {
    // Pass current value of reconfigurable parameter plan_execution/record_trajectory_state_frequency
    double sampling_frequency = 0.0;
    node_->get_parameter_or("plan_execution.record_trajectory_state_frequency", sampling_frequency, 0.0);
    trajectory_monitor_ = std::make_shared<planning_scene_monitor::TrajectoryMonitor>(
        planning_scene_monitor_->getStateMonitor(), sampling_frequency);
  }

  // start recording trajectory states
  if (trajectory_monitor_)
    trajectory_monitor_->startTrajectoryMonitor();

  // start a trajectory execution thread
  trajectory_execution_manager_->execute(
      [this](const moveit_controller_manager::ExecutionStatus& status) { doneWithTrajectoryExecution(status); },
      [this, &plan](std::size_t index) { successfulTrajectorySegmentExecution(plan, index); });
  // wait for path to be done, while checking that the path does not become invalid
  rclcpp::WallRate r(100);
  path_became_invalid_ = false;
  bool preempt_requested = false;

  while (rclcpp::ok() && !execution_complete_ && !path_became_invalid_)
  {
    r.sleep();
    // check the path if there was an environment update in the meantime
    if (new_scene_update_)
    {
      new_scene_update_ = false;
      std::pair<int, int> current_index = trajectory_execution_manager_->getCurrentExpectedTrajectoryIndex();
      if (!isRemainingPathValid(plan, current_index))
      {
        RCLCPP_INFO(LOGGER, "Trajectory component '%s' is invalid after scene update",
                    plan.plan_components_[current_index.first].description_.c_str());
        path_became_invalid_ = true;
        break;
      }
    }

    preempt_requested = preempt_.checkAndClear();
    if (preempt_requested)
      break;
  }

  // stop execution if needed
  if (preempt_requested)
  {
    RCLCPP_INFO(LOGGER, "Stopping execution due to preempt request");
    trajectory_execution_manager_->stopExecution();
  }
  else if (path_became_invalid_)
  {
    RCLCPP_INFO(LOGGER, "Stopping execution because the path to execute became invalid"
                        "(probably the environment changed)");
    trajectory_execution_manager_->stopExecution();
  }
  else if (!execution_complete_)
  {
    RCLCPP_WARN(LOGGER, "Stopping execution due to unknown reason."
                        "Possibly the node is about to shut down.");
    trajectory_execution_manager_->stopExecution();
  }

  // stop recording trajectory states
  if (trajectory_monitor_)
  {
    trajectory_monitor_->stopTrajectoryMonitor();
    plan.executed_trajectory_ =
        std::make_shared<robot_trajectory::RobotTrajectory>(planning_scene_monitor_->getRobotModel(), "");
    trajectory_monitor_->swapTrajectory(*plan.executed_trajectory_);
  }

  // decide return value
  if (path_became_invalid_)
    result.val = moveit_msgs::msg::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE;
  else
  {
    if (preempt_requested)
    {
      result.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
    }
    else
    {
      if (trajectory_execution_manager_->getLastExecutionStatus() ==
          moveit_controller_manager::ExecutionStatus::SUCCEEDED)
        result.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      else if (trajectory_execution_manager_->getLastExecutionStatus() ==
               moveit_controller_manager::ExecutionStatus::TIMED_OUT)
        result.val = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
      else
        result.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
    }
  }
  return result;
}

void plan_execution::PlanExecution::planningSceneUpdatedCallback(
    const planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  if (update_type & (planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY |
                     planning_scene_monitor::PlanningSceneMonitor::UPDATE_TRANSFORMS))
    new_scene_update_ = true;
}

void plan_execution::PlanExecution::doneWithTrajectoryExecution(
    const moveit_controller_manager::ExecutionStatus& /*status*/)
{
  execution_complete_ = true;
}

void plan_execution::PlanExecution::successfulTrajectorySegmentExecution(const ExecutableMotionPlan& plan,
                                                                         std::size_t index)
{
  if (plan.plan_components_.empty())
  {
    RCLCPP_WARN(LOGGER, "Length of provided motion plan is zero.");
    return;
  }

  // if any side-effects are associated to the trajectory part that just completed, execute them
  RCLCPP_DEBUG(LOGGER, "Completed '%s'", plan.plan_components_[index].description_.c_str());
  if (plan.plan_components_[index].effect_on_success_)
    if (!plan.plan_components_[index].effect_on_success_(&plan))
    {
      // execution of side-effect failed
      RCLCPP_ERROR(LOGGER, "Execution of path-completion side-effect failed. Preempting.");
      preempt_.request();
      return;
    }

  // if there is a next trajectory, check it for validity, before we start execution
  ++index;
  if (index < plan.plan_components_.size() && plan.plan_components_[index].trajectory_ &&
      !plan.plan_components_[index].trajectory_->empty())
  {
    std::pair<int, int> next_index(static_cast<int>(index), 0);
    if (!isRemainingPathValid(plan, next_index))
    {
      RCLCPP_INFO(LOGGER, "Upcoming trajectory component '%s' is invalid",
                  plan.plan_components_[next_index.first].description_.c_str());
      path_became_invalid_ = true;
    }
  }
}
