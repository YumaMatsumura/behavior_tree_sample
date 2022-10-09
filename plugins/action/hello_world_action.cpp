#include <behaviortree_cpp_v3/action_node.h>

class HelloWorldAction : public BT::SyncActionNode
{
public:
  HelloWorldAction(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
  {
    std::cout << this->name() << ": create " << std::endl; 
  }
  
  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<std::string>("action") };
  }
  
  // アクションの実行
  BT::NodeStatus tick() override
  {
    std::cout << "Hello World!" << std::endl;
    
    return BT::NodeStatus::SUCCESS;
  }
};

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<HelloWorldAction>("HelloWorldAction");
}

