#include <iostream>
#include "behaviortree_ros2/bt_action_node.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include "./include/srm1/action/act.hpp"

using Act = srm1::action::Act;

class FindObj: public BT::RosActionNode<Act>{
    //Costructor
    public :
        FindObj(const std::string &name,
                const BT::NodeConfig &conf,
                const BT::RosNodeParams &params) : BT::RosActionNode<Act>(name,conf,params){
        }
    // Ports to read data from the blackboard
    static BT::PortsList providedPorts(){
        return providedBasicPorts({BT::InputPort<unsigned>("order")});
    }
    // This is called when the TreeNode is ticked and it should
    // send the request to the action server
    bool setGoal(RosActionNode::Goal& goal) override{
        // get the "request" from the Input port
        getInput("request", goal.request);
        
        //TODO: Locate the pallet we need to unload
        // return true, if we were able to set the goal correctly.
        return true;
    }
    // Callback executed when the reply is received.
    // Based on the reply you may decide to return SUCCESS or FAILURE.
    BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
        std::stringstream ss;
        ss << "Result received: " << wr.result->response;
        // Set up a Schroedinger shared pointer, 
        // because we don't know if the weak pointer is alive
        if(auto shared_node = node_.lock()){
            RCLCPP_INFO(shared_node->get_logger(), ss.str().c_str());
            return BT::NodeStatus::SUCCESS;
        }
        else {
            perror("Unable to safely access weak node ptr.\n");
            return BT::NodeStatus::FAILURE;
        }   
    }
    // Callback invoked when there was an error at the level
    // of the communication between client and server.
    // This will set the status of the TreeNode to either SUCCESS or FAILURE,
    // based on the return value.
    // If not overridden, it will return FAILURE by default.
    virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override{
        // Set up a Schroedinger shared pointer, 
        // because we don't know if the weak pointer is alive
        if(auto shared_node = node_.lock()){
            RCLCPP_ERROR(shared_node->get_logger(), "Error: %d", error);
            return BT::NodeStatus::SUCCESS;
        }
        else {
            perror("Unable to safely access weak node ptr.\n");
            return BT::NodeStatus::FAILURE;
        }   
    }

    // we also support a callback for the feedback, as in
    // the original tutorial.
    // Usually, this callback should return RUNNING, but you
    // might decide, based on the value of the feedback, to abort
    // the action, and consider the TreeNode completed.
    // In that case, return SUCCESS or FAILURE.
    // The Cancel request will be send automatically to the server.
    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback){
        std::stringstream ss;
        ss << "Feedback received: "<<feedback;
        
        // Set up a Schroedinger shared pointer, 
        // because we don't know if the weak pointer is alive
        if(auto shared_node = node_.lock()){
            RCLCPP_INFO(shared_node->get_logger(), ss.str().c_str());
            return BT::NodeStatus::SUCCESS;
        }
        else {
            perror("Unable to safely access weak node ptr.\n");
            return BT::NodeStatus::FAILURE;
        }   
        
        return BT::NodeStatus::RUNNING;
    }    
};

class CalculateGoal: public BT::RosActionNode<Act>{
    //Costructor
    public :
        CalculateGoal(const std::string &name,
                const BT::NodeConfig &conf,
                const BT::RosNodeParams &params) : BT::RosActionNode<Act>(name,conf,params){
        }
        
};
class SaySomething: public BT::RosActionNode<Act>{
    //Costructor
    public :
        SaySomething(const std::string &name,
                const BT::NodeConfig &conf,
                const BT::RosNodeParams &params) : BT::RosActionNode<Act>(name,conf,params){
        }
        BT::NodeStatus tick() override{
            std::cout<<"[SaySomething] : \t"<< this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
    // TODO: Implement stuff todo (like FindObj)
};

// Not an action per se, will deal with it in a different manner
class RetryUntilSuccessful: public BT::RetryNode{
    //Costructor
    public :
        RetryUntilSuccessful(const std::string &name) : BT::RetryNode(name,{}){
        }
        BT::NodeStatus tick() override{
            std::cout<<"[RetryUntilSuccessful] : \t"<< this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
};

class Sync: public BT::RosActionNode<Act>{
    //Costructor
    public :
        Sync(const std::string &name,
                const BT::NodeConfig &conf,
                const BT::RosNodeParams &params) : BT::RosActionNode<Act>(name,conf,params){
        }
};
class ApproachObject: public BT::RosActionNode<Act>{
    //Costructor
    public :
        ApproachObject(const std::string &name,
                const BT::NodeConfig &conf,
                const BT::RosNodeParams &params) : BT::RosActionNode<Act>(name,conf,params){
        }
};
