#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <thread>
#include "../include/my_bt_moveit/move_base_action.hpp"

class BehaviorTreeNode : public rclcpp::Node {
public:
    BehaviorTreeNode() : Node("bt_moveit_node") {
        RCLCPP_INFO(this->get_logger(), "Behavior Tree Node Started");

        // Creazione del Behavior Tree Factory
        BT::BehaviorTreeFactory factory;

        // Registriamo il nodo di azione MoveBase
        factory.registerNodeType<MoveBaseAction>("MoveBase");

        // Carichiamo il Behavior Tree dal file XML
        auto tree = factory.createTreeFromFile(
            this->declare_parameter<std::string>("bt_xml_path", 
            "src/my_bt_moveit/behavior_trees/behavior_tree.xml"));

        // Esegui il Behavior Tree con un loop
        while (rclcpp::ok()) {
            BT::NodeStatus status = tree.tickWhileRunning();
            if (status == BT::NodeStatus::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Behavior Tree Success!");
                break;
            } else if (status == BT::NodeStatus::FAILURE) {
                RCLCPP_ERROR(this->get_logger(), "Behavior Tree Failed!");
                break;
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BehaviorTreeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}