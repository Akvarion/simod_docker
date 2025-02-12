#include <iostream>
#include <behaviortree_cpp/behavior_tree.h>

class FindObj: public BT::SyncActionNode{
    //Costructor
    public :
        FindObj(const std::string &name) : BT::SyncActionNode(name,{}){
        }
        BT::NodeStatus tick() override{
            std::cout<<"[FindObj] : \t"<< this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
};
class CalculateGoal: public BT::SyncActionNode{
    //Costructor
    public :
        CalculateGoal(const std::string &name) : BT::SyncActionNode(name,{}){
        }
        BT::NodeStatus tick() override{
            std::cout<<"[CalculateGoal] : \t"<< this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
};
class SaySomething: public BT::SyncActionNode{
    //Costructor
    public :
        SaySomething(const std::string &name) : BT::SyncActionNode(name,{}){
        }
        BT::NodeStatus tick() override{
            std::cout<<"[SaySomething] : \t"<< this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
};
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
class Sync: public BT::SyncActionNode{
    //Costructor
    public :
        Sync(const std::string &name) : BT::SyncActionNode(name,{}){
        }
        BT::NodeStatus tick() override{
            std::cout<<"[Sync] : \t"<< this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
};
class ApproachObject: public BT::SyncActionNode{
    //Costructor
    public :
        ApproachObject(const std::string &name) : BT::SyncActionNode(name,{}){
        }
        BT::NodeStatus tick() override{
            std::cout<<"[ApproachObject] : \t"<< this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
};
