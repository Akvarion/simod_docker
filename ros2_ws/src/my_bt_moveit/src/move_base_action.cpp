#include "../include/my_bt_moveit/move_base_action.hpp"

MoveBaseAction::MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList MoveBaseAction::providedPorts() {
    return { 
        BT::InputPort<double>("target_x"),
        BT::InputPort<double>("target_y"),
        BT::InputPort<double>("target_theta")
    };
}

BT::NodeStatus MoveBaseAction::tick() {
    double x, y, theta;
    if (!getInput("target_x", x) || !getInput("target_y", y) || !getInput("target_theta", theta)) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveBaseAction"), "Missing input ports");
        return BT::NodeStatus::FAILURE;
    }

    // Creiamo un nodo ROS2 per pubblicare comandi di velocità
    auto node = rclcpp::Node::make_shared("move_base_action_node");
    auto cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>("/right_summit/cmd_vel", 10);

    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = x;
    cmd_msg.angular.z = theta;

    RCLCPP_INFO(node->get_logger(), "Moving to x: %.2f, y: %.2f, theta: %.2f", x, y, theta);
    
    // Pubblica il messaggio per far muovere il robot
    cmd_pub->publish(cmd_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(2000));  // Simula il movimento

    return BT::NodeStatus::SUCCESS;
}
