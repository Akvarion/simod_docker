#include <memory>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include <simod_moveit_action_controller/action/multi_dof_follow_joint_trajectory.hpp>

using MultiDofFollowJointTrajectory =
    simod_moveit_action_controller::action::MultiDofFollowJointTrajectory;
using GoalHandleMultiDofFollowJointTrajectory =
    rclcpp_action::ServerGoalHandle<MultiDofFollowJointTrajectory>;

class MultiDofActionController : public rclcpp::Node {
  public:
    MultiDofActionController() : Node("multi_dof_action_controller") {
      using namespace std::placeholders;  

      action_server_ = rclcpp_action::create_server<MultiDofFollowJointTrajectory>(
          this,
          "multi_dof_joint_trajectory_action",   // must match your MoveIt controller YAML
          std::bind(&MultiDofActionController::handle_goal, this, _1, _2),
          std::bind(&MultiDofActionController::handle_cancel, this, _1),
          std::bind(&MultiDofActionController::handle_accepted, this, _1));

      // Publisher for waypoints or robot interface
      publisher_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
          "desired_waypoints", 10);

      RCLCPP_INFO(this->get_logger(), "MultiDOF Action Controller ready.");
    }

  private:
    rclcpp_action::Server<MultiDofFollowJointTrajectory>::SharedPtr action_server_;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr publisher_;

    // ---- Action callbacks ----------------------------------------------------

    rclcpp_action::GoalResponse handle_goal( 
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MultiDofFollowJointTrajectory::Goal> goal){
          (void)uuid;
          if (goal->trajectory.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Rejected goal: empty trajectory.");
            return rclcpp_action::GoalResponse::REJECT;
          }
          RCLCPP_INFO(this->get_logger(), "Accepted trajectory with %zu points",
                      goal->trajectory.points.size());
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMultiDofFollowJointTrajectory> goal_handle){
          RCLCPP_INFO(this->get_logger(), "Cancel request received.");
          (void)goal_handle;
          return rclcpp_action::CancelResponse::ACCEPT;
        }

    void handle_accepted(
        const std::shared_ptr<GoalHandleMultiDofFollowJointTrajectory> goal_handle)
    {
      std::thread{std::bind(&MultiDofActionController::execute, this, std::placeholders::_1),
                  goal_handle}
          .detach();
    }

    // ---- Execution -----------------------------------------------------------

    void execute(const std::shared_ptr<GoalHandleMultiDofFollowJointTrajectory> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Executing MultiDOF trajectory.");

      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<MultiDofFollowJointTrajectory::Feedback>();
      auto result   = std::make_shared<MultiDofFollowJointTrajectory::Result>();

      auto start_time = this->now();
      rclcpp::Rate loop_rate(50.0);

      for (size_t i = 0; i < goal->trajectory.points.size(); ++i)
      {
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
          RCLCPP_WARN(this->get_logger(), "Goal canceled during execution.");
          return;
        }

        // Wait until time_from_start is reached
        auto elapsed = (this->now() - start_time);
        auto target_time = rclcpp::Duration(goal->trajectory.points[i].time_from_start);
        if (elapsed < target_time) {
          loop_rate.sleep();
          --i;
          continue;
        }

        // Publish desired waypoint (to Gazebo or low-level controller)
        publisher_->publish(goal->trajectory.points[i]);

        // Provide feedback
        feedback->desired = goal->trajectory.points[i];
        feedback->actual  = goal->trajectory.points[i];  // replace with odom feedback later
        feedback->error   = trajectory_msgs::msg::MultiDOFJointTrajectoryPoint();
        goal_handle->publish_feedback(feedback);
      }

      result->error_code = result->SUCCESSFUL;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Trajectory execution finished successfully.");
    }
};

// ---- main -----------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiDofActionController>());
  rclcpp::shutdown();
  return 0;
}