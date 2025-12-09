#ifndef MOVE_BASE_ACTION_HPP
#define MOVE_BASE_ACTION_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>

void blackboard_print(std::shared_ptr<BT::Blackboard> blackboard);

class MoveBaseAction : public BT::SyncActionNode {
public:
    MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class StopBaseAction : public BT::SyncActionNode {
public:
    StopBaseAction(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

#endif // MOVE_BASE_ACTION_HPP