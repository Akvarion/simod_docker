#ifndef MOVE_BASE_ACTION_HPP
#define MOVE_BASE_ACTION_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MoveBaseAction : public BT::SyncActionNode {
public:
    MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

#endif // MOVE_BASE_ACTION_HPP