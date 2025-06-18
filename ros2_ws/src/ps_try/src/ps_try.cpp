#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ps_try");
namespace mtc = moveit::task_constructor;

// Class representing the MTC task node, with methods to set up the planning scene and execute the task.
class MTCTaskNode{
  public:
    MTCTaskNode(const rclcpp::NodeOptions& options) : 
      node_{std::make_shared<rclcpp::Node>("ps_try_mtc_node", options)} {
    };

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface(){
      return node_->get_node_base_interface();
    };

    void doTask();

    void setupPlanningScene();

  private:
    // Compose an MTC task from a series of stages.
    mtc::Task createTask(const std::string *robot_side);
    mtc::Task task_;
    std::string robot_side;
    rclcpp::Node::SharedPtr node_;
};

// This should set up an empty planning scene.
void MTCTaskNode::setupPlanningScene(){
  // Create a planning scene interface
  moveit::planning_interface::PlanningSceneInterface psi;

  // Log the setup process
  RCLCPP_INFO(LOGGER, "Setting up the planning scene...");
  // Load the robot model (URDF and SRDF)
  auto robot_description = node_->get_parameter("robot_description").as_string();
    if (robot_description.empty()) {
        RCLCPP_ERROR(LOGGER, "Robot description (URDF) is not loaded. Check your parameters.");
        return;
    }
  // Optionally, you can load additional robot-specific parameters here
  RCLCPP_INFO(LOGGER, "Robot description loaded successfully.");
  
  // Create a PlanningScene message
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = false; // Indicates that this is a diff update to the planning scene
  
  // Publish the planning scene
  RCLCPP_INFO(LOGGER, "Publishing the planning scene...");
  psi.applyPlanningScene(planning_scene_msg);
  RCLCPP_INFO(LOGGER, "Planning scene setup complete.");

}
// Create an MTC task with stages for the specified robot_side which can be "l" or "r",
// left or right respectively. In hindsight, this should be a character instead of a string, todo.
mtc::Task MTCTaskNode::createTask(const std::string *robot_side){
  mtc::Task task;
  // Load the robot model from a URDF file into a buffer
  std::ifstream t ("/ros2_ws/"+*robot_side+"_resolved.urdf");
  std::stringstream buffer;
  buffer << t.rdbuf();
  
  task.stages()->setName("demo task");
  task.loadRobotModel(node_, buffer.str());

  // define robot groups name
  // They should match the names defined in the URDF/SRDF files
  const auto& base_group_name = "srm_"+*robot_side+"_base";
  const auto& ur_group_name = "srm_"+*robot_side+"_ur";
  const auto& ee_frame = "srm_"+*robot_side+"_ee";

  // Set task properties
  task.setProperty("group", base_group_name);
  task.setProperty("group", ur_group_name);
  task.setProperty("ik_frame", ee_frame);

 // Disable warnings for this line, as it's a variable that's set but not used in this example
 #pragma GCC diagnostic push
 #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage *current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
 #pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);
  
  return task;
}

int main(int argc, char ** argv){

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
