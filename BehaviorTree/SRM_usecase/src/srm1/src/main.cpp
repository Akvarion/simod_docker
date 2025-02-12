#include <iostream>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include "callbacks.hpp"

using namespace BT;
/*
    1 Define nodes: override tick() method
    2 Define Statuses
    3 XML load / define
    4 main: 
        a) register nodes and conditions through factory
        b) load tree
        c) start tick

*/

//Define GripperInterface for Gripper interactions,
//Might be useful for later.
class GripperInterface {
    private: bool isOpen; 
    
    public:
        GripperInterface() : isOpen(true){}
        
        NodeStatus open(){
            isOpen=true;
            std::cout<<"[Gripper: OPEN]"<<std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        NodeStatus close(){
            isOpen=false;
            std::cout<<"[Gripper: CLOSE]"<<std::endl;
            return BT::NodeStatus::SUCCESS; 
        }    
};


//Condition Nodes
BT::NodeStatus NearObj(){
    std::cout<<"[Near Ball: NO]"<<std::endl;
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GotObj(){
    std::cout<<"[Grasped the Ball: NO]"<<std::endl;
    return BT::NodeStatus::FAILURE;
}


int main(void){
    BehaviorTreeFactory factory;

    // Register custom action nodes with the factory
    factory.registerNodeType<FindObj>("FindObj");
    factory.registerNodeType<CalculateGoal>("CalculateGoal");
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<RetryUntilSuccessful>("RetryUntilSuccessful");
    factory.registerNodeType<Sync>("Sync");
    factory.registerNodeType<ApproachObject>("ApproachObject");
    
    // Register custom condition Nodes
    factory.registerSimpleCondition("BallClose", std::bind(NearObj));
    factory.registerSimpleCondition("BallGrasped", std::bind(GotObj));

    //Istance of GripperInterface
    GripperInterface gripper;

    // Register a simple action node using the GripperInterface instance
    factory.registerSimpleAction("GraspBall", std::bind(&GripperInterface::close, &gripper));

    // Create BehavTree from file SRM1
    auto tree= factory.createTreeFromFile("./SRM1.xml");

    //RUN
    tree.tickWhileRunning();

    return 0;
}