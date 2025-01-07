#include <iostream>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

using namespace BT;

// Define FindBall action node
class FindBall : public BT::SyncActionNode{
    //Costructor
    public :
        FindBall(const std::string &name) : BT::SyncActionNode(name,{}){

        }
        NodeStatus tick() override{
            std::cout<<"[FindBall] : \t"<< this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
};

//Define PickBall Action Node
class PickBall : public BT::SyncActionNode{
    public :
        PickBall(const std::string &name) : BT::SyncActionNode(name,{}){
            //void
        }
        NodeStatus tick() override{
            std::cout<< "[PickBall] : \t" << this->name() << std::endl;
            return BT::NodeStatus::SUCCESS; 
        }

};

class PlaceBall : public BT::SyncActionNode{
    public :
        PlaceBall(const std::string &name) : BT::SyncActionNode(name,{}){
            //void
        }
        NodeStatus tick() override{
            std::cout<<"[PlaceBall] : \t"<<this->name()<<std::endl;
            return BT::NodeStatus::SUCCESS;
        }

};

//Define GripperInterface for Gripper interactions
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

BT::NodeStatus NearBall(){
    std::cout<<"[Near Ball: NO]"<<std::endl;
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GotBall(){
    std::cout<<"[Grasped the Ball: NO]"<<std::endl;
    return BT::NodeStatus::FAILURE;
}

// Define the behavior tree structure in XML format
static const char* xml_text = R"(
 <root BTCPP_format="4" main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <FindBall   name="found_ok"/>
                <Sequence>
                    <Fallback>
                        <BallClose   name="no_ball"/>
                        <PickBall    name="approach_ball"/>
                    </Fallback>
                    <Fallback>
                        <BallGrasped   name="no_grasp"/>
                        <GraspBall  name="grasp_ball"/>
                    </Fallback>
                </Sequence>
            <PlaceBall   name="ball_placed"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

 int main(void){
    BehaviorTreeFactory factory;

    // Register custom action nodes with the factory
    factory.registerNodeType<FindBall>("FindBall");
    factory.registerNodeType<PickBall>("PickBall");
    factory.registerNodeType<PlaceBall>("PlaceBall");

    // Register custom condition Nodes
    factory.registerSimpleCondition("BallClose", std::bind(NearBall));
    factory.registerSimpleCondition("BallGrasped", std::bind(GotBall));

    //Istance of GripperInterface
    GripperInterface gripper;

    // Register a simple action node using the GripperInterface instance
    factory.registerSimpleAction("GraspBall", std::bind(&GripperInterface::close, &gripper));

    // Create BehavTree
    auto tree= factory.createTreeFromText(xml_text);

    //RUN
    tree.tickWhileRunning();

    return 0;
 }