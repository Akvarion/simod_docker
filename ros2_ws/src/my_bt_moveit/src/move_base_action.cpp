#include "../include/my_bt_moveit/move_base_action.hpp"

std::string right_cmd_vel="/right_summit_cmd_vel";
std::string left_cmd_vel="/left_summit_cmd_vel";

void blackboard_print(std::shared_ptr<BT::Blackboard> blackboard){
    std::cout << "\nBlackboard\n" << std::endl;
    for (const auto& key : blackboard->getKeys()) {
        std::cout << key << ": " << blackboard->get<std::string>(std::string(key)) << std::endl;
    }
}

MoveBaseAction::MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList MoveBaseAction::providedPorts() {
    return { 
        BT::InputPort<std::vector<double>>("APPROACH_LEFT_BASE_XY_VEL"),
        BT::InputPort<std::vector<double>>("APPROACH_RIGHT_BASE_XY_VEL"),
        BT::InputPort<double>("APPROACH_TIME"),

    };
}

BT::NodeStatus MoveBaseAction::tick() {
    // SIMULATING CONTROL LAYER

    std::vector<double> right_xy_vel,left_xy_vel;
    double APPROACH_TIME;
   
    // DEBUG
    //auto blackboard = config().blackboard;
    //blackboard_print(blackboard);

    if (!getInput("APPROACH_RIGHT_BASE_XY_VEL", right_xy_vel) || !getInput("APPROACH_LEFT_BASE_XY_VEL", left_xy_vel)) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveBaseAction"), "Missing approach input ports");
        return BT::NodeStatus::FAILURE;
    }
    if (!getInput("APPROACH_TIME",APPROACH_TIME)) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveBaseAction"), "Missing APPROACH_TIME input port");
        return BT::NodeStatus::FAILURE;
    }
   
    // MAKE ROS2 PUB
    auto node = rclcpp::Node::make_shared("move_base_action_node");
    auto cmd_pub_right = node->create_publisher<geometry_msgs::msg::Twist>(right_cmd_vel, 10);
    auto cmd_pub_left = node->create_publisher<geometry_msgs::msg::Twist>(left_cmd_vel, 10);

    // BUILD TWIST MESSAGE, ONLY RIGHT BASE FOR NOW
    geometry_msgs::msg::Twist cmd_msg_left,cmd_msg_right;
    cmd_msg_right.linear.x = right_xy_vel[0];
    cmd_msg_right.linear.y = right_xy_vel[1];
    cmd_msg_right.angular.z = 0.0;
    cmd_msg_left.linear.x = left_xy_vel[0];
    cmd_msg_left.linear.y = left_xy_vel[1];
    cmd_msg_left.angular.z = 0.0;

    RCLCPP_INFO(node->get_logger(), "Moving right base with x: %.4f, y: %.4f, theta: %.4f", right_xy_vel[0], right_xy_vel[1], 0.0);
    RCLCPP_INFO(node->get_logger(), "Moving left base with x: %.4f, y: %.4f, theta: %.4f", left_xy_vel[0], left_xy_vel[1], 0.0);
    
    // SET TIME NOW AND DURATION FOR VEL CMD
    rclcpp::Time t0 = node->get_clock()->now();
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(APPROACH_TIME);

    
    // TIME-CONTROL FOR SIMULATION, otherwise inconsistent results 
    while (rclcpp::ok() && (node->get_clock()->now() - t0) < timeout) {
        // PUBLISH
        cmd_pub_right->publish(cmd_msg_right);
        cmd_pub_left->publish(cmd_msg_left);
        rclcpp::spin_some(node);
        // 10 ms steps
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList StopBaseAction::providedPorts() {
    return {};
}


StopBaseAction::StopBaseAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus StopBaseAction::tick() {
    auto node = rclcpp::Node::make_shared("stop_base_action_node");
    
    auto cmd_pub_right = node->create_publisher<geometry_msgs::msg::Twist>(right_cmd_vel, 10);
    auto cmd_pub_left = node->create_publisher<geometry_msgs::msg::Twist>(left_cmd_vel,10);

    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = 0.0;
    cmd_msg.linear.y = 0.0;
    cmd_msg.angular.z = 0.0;

    // PUBLISH
    // cmd_pub_right->publish(cmd_msg);
    // cmd_pub_left->publish(cmd_msg);

    // TIME-CONTROL FOR SIMULATION, otherwise inconsistent results 
    rclcpp::Time t0 = node->get_clock()->now();
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(2.0);

    while (rclcpp::ok() && (node->get_clock()->now() - t0) < timeout) {
        cmd_pub_right->publish(cmd_msg);
        cmd_pub_left->publish(cmd_msg);
        rclcpp::spin_some(node);
        // 10 ms steps
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return BT::NodeStatus::SUCCESS;
}