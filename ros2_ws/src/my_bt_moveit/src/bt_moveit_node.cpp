#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <yaml-cpp/yaml.h>
#include "../include/my_bt_moveit/move_base_action.hpp"

static int8_t populate_blackboard_from_yaml(std::shared_ptr<BT::Blackboard> blackboard){
    YAML::Node root = YAML::LoadFile("/ros2_ws/src/my_bt_moveit/config/blackboard.yaml");
    for (YAML::const_iterator it = root.begin(); it != root.end(); ++it) {
        if(it->second.IsScalar()){
            try{
                blackboard->set(it->first.as<std::string>(),it->second.as<double>());
        
            } catch (YAML::Exception e){
                RCLCPP_ERROR(rclcpp::get_logger("populate_blackboard_from_yaml"), "YAML Exception: %s", e.what());
                return EXIT_FAILURE;
            }  
        }
        if(it->second.IsSequence()){
            blackboard->set(it->first.as<std::string>(),it->second.as<std::vector<double>>());
        }
    }
    return EXIT_SUCCESS;
}


static void populateBlackboard(std::shared_ptr<BT::Blackboard> blackboard){
    
    // Generic/common mission data (coordinator-owned)
    blackboard->set("timeout_s", 30.0); 
    // Namespaced defaults for left, right and stop behaviours
    blackboard->set("left_approach_linear_x",  0.0022);
    blackboard->set("left_approach_linear_y",  0.25);
    blackboard->set("left_approach_angular_z", 0.01);

    blackboard->set("right_approach_linear_x", -0.0022); // X reversed for right
    blackboard->set("right_approach_linear_y",  0.25);
    blackboard->set("right_approach_angular_z", 2.00);
    
    // Example inputs for MoveBaseAction (adapt keys & types to your node ports)
    // These keys should match the BT XML port remapping or the port names expected by the node.


    // // Example inputs for a MoveArm node: vector of joint positions
    // std::vector<double> arm_joints = {0.0, -1.0, 0.5, 0.0, 0.0, 0.0};
    // bb->set("arm_joint_values", arm_joints);

    blackboard->set("left_base_topic", "/left_summit_cmd_vel");
    blackboard->set("right_base_topic", "/right_summit_cmd_vel");

    blackboard->set("APPROACH_TIME", 10.0);
    // You can extend this routine to set different defaults depending on your tree content.
    // If you want node-type-based population, inspect the tree or maintain a mapping here
    // and set only the relevant keys. Keeping the coordinator responsible for data
    // keeps behavior nodes stateless and reusable.
}



class BehaviorTreeNode : public rclcpp::Node {
public:
    BehaviorTreeNode() : Node("bt_moveit_node") {
        RCLCPP_INFO(this->get_logger(), "Behavior Tree Node Started");
        auto blackboard = BT::Blackboard::create();
        if(populate_blackboard_from_yaml(blackboard) != EXIT_SUCCESS){
            RCLCPP_ERROR(this->get_logger(), "Failed to populate blackboard from YAML, using defaults.");
            //populateBlackboard(blackboard);
        }
        // Creazione del Behavior Tree Factory
        BT::BehaviorTreeFactory factory;

        // Registriamo il nodo di azione MoveBase and StopBase
        factory.registerNodeType<MoveBaseAction>("MoveBase");
        factory.registerNodeType<StopBaseAction>("StopBase");

        // Carichiamo il Behavior Tree dal file XML
        auto tree = factory.createTreeFromFile(
            this->declare_parameter<std::string>("bt_xml_path", 
            "src/my_bt_moveit/behavior_trees/behavior_tree.xml"), blackboard);
        
        // blackboard_print(blackboard);
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