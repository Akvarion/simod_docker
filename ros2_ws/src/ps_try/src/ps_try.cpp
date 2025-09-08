#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <urdf_parser/urdf_parser.h> // Provided by urdfdom
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

std::map<std::string, double> compute_IK(const moveit::core::RobotModelPtr robot_model,
                                          const std::string &robot_group,
                                          const geometry_msgs::msg::PoseStamped &pose) {
    // This function should compute the IK for the given pose and robot group
    // For now, we return an empty map as a placeholder
    std::map<std::string, double> joint_goal;
    moveit::core::RobotState robot_state(robot_model);
    const moveit::core::JointModelGroup *jmg = robot_state.getJointModelGroup(robot_group);
    if (!jmg) {
      RCLCPP_ERROR(LOGGER, "JointModelGroup '%s' not found!", robot_group.c_str());
      return joint_goal;
    }
    bool found_ik = robot_state.setFromIK(jmg, pose.pose, pose.header.frame_id, 5.0);
    if(!found_ik) {
        RCLCPP_ERROR(LOGGER, "IK solution not found for group '%s'.", robot_group.c_str());
        return joint_goal;
    } else {
      std::vector<double> joint_positions;
      robot_state.copyJointGroupPositions(jmg, joint_positions);
      // Map joint names to values
      const auto& joint_names = jmg->getVariableNames();
      for (size_t i = 0; i < joint_names.size(); ++i){
          joint_goal[joint_names[i]] = joint_positions[i];
      }
    }
    // Fill joint_goal with computed IK values
    return joint_goal;
}

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
    std::string robot_side;
  
    int print_name(void){
      RCLCPP_INFO(LOGGER, "Node name: %s", node_->get_name());
      return 0;
    }
  private:
    // Compose an MTC task from a series of stages.
    mtc::Task createTask(void);
    mtc::Task task_;
    
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
mtc::Task MTCTaskNode::createTask(){
  mtc::Task task;
  // std::string robot_side_full = (*robot_side == "l") ? "left" : "right";
  // Load the robot model from a URDF file into a buffer
  // std::ifstream t ("/ros2_ws/src/simod_proj/ur/xacro/srm.urdf");
  // std::stringstream buffer;
  // buffer << t.rdbuf();

  //std::string robot_description = node_->get_parameter("robot_description").as_string();
  
  RCLCPP_INFO(LOGGER, "READING DUAL ROBOT MODEL...");
  task.loadRobotModel(node_, "robot_description");
  RCLCPP_INFO(LOGGER, "DUAL ROBOT MODEL LOADED.");
  task.setName("Dual_System_Trial_01");
  
  // Define the namespaces
  const auto& left_ns = "/left";
  const auto& right_ns = "/right";

  // define robot groups name
  // They should match the names defined in the URDF/SRDF files
  // const auto &base_group_name = "srm_"+*robot_side+"_base";
  // const auto &ur_group_name = "srm_"+*robot_side+"_ur";
  // const auto &ee_frame = "srm_"+*robot_side+"_ee";

  // const auto &srm_station = "srm_" + *robot_side;

  const auto& left_arm_group = "srm_l_ur";
  const auto& right_arm_group = "srm_r_ur";
  const auto& left_base_group = "srm_base_l";
  const auto& right_base_group = "srm_base_r";  
  
  const auto& dual_arms_group = "dual_arms";
  const auto& dual_bases_group = "dual_bases";

  // Set task properties
  // task.setProperty("base_group", base_group_name);
  // task.setProperty("ur_group", ur_group_name);
  // task.setProperty("ee_group", ee_frame);

 // Disable warnings for this line, as it's a variable that's set but not used in this example
 #pragma GCC diagnostic push
 #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage *current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
 #pragma GCC diagnostic pop

  // auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  // current_state_ptr = stage_state_current.get();
  // task.add(std::move(stage_state_current));

  // Setting up planner awareness
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setProperty("goal_joint_tolerance", 0.001);

  // auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(.01);
  
// LOAD SINGLE ROBOT DESCRIPTIONS FOR PLANNING.
// This should be seprated from here as it may not be "clean".
  //LOADING LEFT ROBOT URDF FROM PARAMETER.
  std::string left_urdf;
  // try {
  //   node_->get_parameter("/left/robot_description", left_urdf);
  // } catch (const std::exception &e) {
  //   RCLCPP_ERROR(LOGGER, "Failed to get left robot description: %s", e.what());
  //   exit(-1);  // Terminate program due to critical failure
  // }


  //LOADING CUSTOM SEMANTICS LEFT, "HALF" OF THE DUAL ROBOT SRDF
  std::ifstream srdf_file_l("/ros2_ws/src/simod_proj/ur/xacro/left_semantic.srdf");
  std::stringstream srdf_buffer;
  srdf_buffer << srdf_file_l.rdbuf();
  std::string srdf_string_l = srdf_buffer.str();
  // Reset stringstream for reuse
  srdf_buffer.str("");
  
  //LOADING URDF LEFT, "HALF" OF THE DUAL ROBOT URDF
  std::ifstream urdf_file_l("/ros2_ws/left_resolved.urdf");
  std::stringstream urdf_buffer;
  urdf_buffer << urdf_file_l.rdbuf();
  std::string urdf_string_l = urdf_buffer.str();
  // Reset stringstream for reuse
  urdf_buffer.str("");

  //PARSING
  RCLCPP_INFO(LOGGER, "Parsing left URDF...");
  urdf::ModelInterfaceSharedPtr urdf_model_left = urdf::parseURDF(urdf_string_l);
  RCLCPP_INFO(LOGGER, "Parsing complete.");
  RCLCPP_INFO(LOGGER, "INITIALIZING MODEL...");

  srdf::ModelSharedPtr srdf_model_left = std::make_shared<srdf::Model>();
  
  if(srdf_model_left->initFile(*urdf_model_left, "/ros2_ws/src/simod_proj/ur/xacro/left_semantic.srdf")){
    RCLCPP_INFO(LOGGER, "LEFT SRDF INITIALIZATION COMPLETE.");
  }
  else {
    RCLCPP_ERROR(LOGGER, "LEFT SRDF INITIALIZATION FAILED.");
  }

  // Build the MoveIt RobotModel
  RCLCPP_INFO(LOGGER, "CREATING LEFT ROBOT MODEL...");
  moveit::core::RobotModelPtr left_robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model_left, srdf_model_left);
  RCLCPP_INFO(LOGGER, "LEFT ROBOT MODEL CREATED.");
  //Load right robot
  std::string right_urdf; 
  // try {
  //   node_->get_parameter("/right/robot_description", right_urdf);
  // } catch (const std::exception &e) {
  //   RCLCPP_ERROR(LOGGER, "Failed to get right robot description: %s", e.what());
  //   exit(-1);  // Terminate program due to critical failure
  // }
  
  //LOADING CUSTOM SEMANTICS RIGHT, "HALF" OF THE DUAL ROBOT SRDF
  std::ifstream srdf_file_r("/ros2_ws/src/simod_proj/ur/xacro/right_semantic.srdf");
  srdf_buffer << srdf_file_r.rdbuf();
  std::string srdf_string_r = srdf_buffer.str();

  // LOADING URDF RIGHT, "HALF" OF THE DUAL ROBOT URDF
  std::ifstream urdf_file_r("/ros2_ws/right_resolved.urdf");
  urdf_buffer << urdf_file_r.rdbuf();
  std::string urdf_string_r = urdf_buffer.str();

  //PARSING
  RCLCPP_INFO(LOGGER, "Parsing right URDF...");
  urdf::ModelInterfaceSharedPtr urdf_model_right = urdf::parseURDF(urdf_string_r);
  RCLCPP_INFO(LOGGER, "Parsing complete.");
  RCLCPP_INFO(LOGGER, "INITIALIZING MODEL...");
  srdf::ModelSharedPtr srdf_model_right = std::make_shared<srdf::Model>();

  if(srdf_model_right->initString(*urdf_model_right, srdf_string_r)){
    RCLCPP_INFO(LOGGER, "RIGHT SRDF INITIALIZATION COMPLETE.");
  }
  else {
    RCLCPP_ERROR(LOGGER, "RIGHT SRDF INITIALIZATION FAILED.");
  }

  // Build the MoveIt RobotModel
  RCLCPP_INFO(LOGGER, "CREATING RIGHT ROBOT MODEL...");
  moveit::core::RobotModelPtr right_robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model_right, srdf_model_right);


  RCLCPP_INFO(LOGGER, "!ROBOT INITIALIZATION COMPLETE!");
  // STAGE 1
  // Current state stage needs to read from namespaced joint states
  {
    auto stage_current_state = std::make_unique<mtc::stages::CurrentState>("current_state");
    stage_current_state->properties().set("joint_state_topic", "/joint_states");  // Using merged joint states
    task.add(std::move(stage_current_state));
  }
  // STAGE 2
  // Move Bases
  {
    // Set up the MoveTo stage for left base
    auto stage_base_left = std::make_unique<mtc::stages::MoveTo>("move_base_left",sampling_planner);
    // Set stage group to identify the base in the srdf
    stage_base_left->setGroup(left_base_group);
    // Set orientation in the space
    double theta = 90.0;
    // Define pose goal for right and left base
    geometry_msgs::msg::PoseStamped base_goal_right,base_goal_left;
    base_goal_left.header.frame_id = "muraUse1";
    base_goal_left.pose.position.x = -1.0;
    base_goal_left.pose.position.y = 2.0;
    base_goal_left.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(theta/2), cos(theta/2)));
    // Set goal and add the task
    stage_base_left->setGoal(base_goal_left);
    task.add(std::move(stage_base_left));

    // Repeat for right base
    base_goal_right.header.frame_id = "muraUse1";
    base_goal_right.pose.position.x = 1.0;
    base_goal_right.pose.position.y = 2.0;
    base_goal_right.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(-theta/2), cos(-theta/2)));
    auto stage_base_right = std::make_unique<mtc::stages::MoveTo>("move_base_right", sampling_planner);
    stage_base_right->setGroup(right_base_group); // e.g., "srm_base_l"
    stage_base_right->setGoal(base_goal_right);
    task.add(std::move(stage_base_right));

  }

  // STAGE 3
  // Coordinated Dual-Arm Movement
  {
      auto stage_dual_arms = std::make_unique<mtc::stages::MoveTo>("move_dual_arms", sampling_planner);
      
      
      // Set pose goal relative to object in the scene
      geometry_msgs::msg::PoseStamped pose_goal;
      pose_goal.header.frame_id = "pacco_clone_0";  // The object's frame in the planning scene
      pose_goal.pose.position.x = 0.0;         // Offset from the object's origin (e.g., for grasping)
      pose_goal.pose.position.y = 0.1;
      pose_goal.pose.position.z = 0.2;
      pose_goal.pose.orientation.w = 1.0;

      // Calculate map goal for left and right arms
      std::map<std::string,double> *joint_goal;
      std::map<std::string,double> joint_goal_right = compute_IK(right_robot_model,right_arm_group,pose_goal);
      std::map<std::string,double> joint_goal_left = compute_IK(left_robot_model,left_arm_group,pose_goal);
      // Merge solutions
      joint_goal_left.merge(joint_goal_right);
      joint_goal = &joint_goal_left;

      stage_dual_arms->setGroup(dual_arms_group);
      stage_dual_arms->setGoal(*joint_goal);

      task.add(std::move(stage_dual_arms));
  }

  return task;
}

void MTCTaskNode::doTask() {
  // Create a task for the left robot side
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed.");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
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

  mtc_task_node->robot_side = "l";  // Set the robot side to left, can be "l" or "r"
  mtc_task_node->doTask();
  RCLCPP_INFO(LOGGER, "Task execution complete, shutting down...");

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
