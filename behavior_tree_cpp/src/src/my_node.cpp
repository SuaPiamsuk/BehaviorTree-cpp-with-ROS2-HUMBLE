#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

//#define MANUAL_STATIC_LINKING

#ifdef MANUAL_STATIC_LINKING
// #include "dummy_nodes.h"
#endif


using namespace BT;
// BT::NodeStatus CheckBattery();

//--------------------------------------

// Example of custom SyncActionNode (synchronous action)
// without ports.
// class ApproachObject : public BT::SyncActionNode
// {
//   public:
//     ApproachObject(const std::string& name) :
//         BT::SyncActionNode(name, {})
//     {
//     }

//     // You must override the virtual function tick()
//     BT::NodeStatus tick() override;
// };



class TickCount : public BT::CoroActionNode
{
  public:
    TickCount(const std::string& name, const BT::NodeConfiguration& config)
        : BT::CoroActionNode(name, config), tick_count_(0)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("ticks") };
    }

    BT::NodeStatus tick() override
    {
        int ticks;
        if (!getInput("ticks", ticks))
        {
            throw BT::RuntimeError("missing required input [ticks]: ", 
                                    this->name());
        }

        if (tick_count_ < ticks)
        {
            tick_count_++;
            std::cout << this->name() << " is RUNNING. Tick count: " << tick_count_ << std::endl;
            setStatus(BT::NodeStatus::RUNNING);
            return BT::NodeStatus::RUNNING;
        }
        else if (tick_count_ >= ticks)
        {
            std::cout << this->name() << " is SUCCESS. Tick count: " << tick_count_ << std::endl;
            tick_count_ = 0;
            setStatus(BT::NodeStatus::SUCCESS);
            return BT::NodeStatus::SUCCESS;
        }
        std::cout << this->name() << " is FAILURE. Tick count: " << tick_count_ << std::endl;
        setStatus(BT::NodeStatus::FAILURE);
        return BT::NodeStatus::FAILURE;
    }

  private:
    int tick_count_;
};






/** Behavior Tree are used to create a logic to decide what
 * to "do" and when. For this reason, our main building blocks are
 * Actions and Conditions.
 *
 * In this tutorial, we will learn how to create custom ActionNodes.
 * It is important to remember that NodeTree are just a way to
 * invoke callbacks (called tick() ). These callbacks are implemented by the user.
 */

// clang-format off
static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
          <TickCount name="tick_4" ticks="3"/>
          <TickCount name="tick_2" ticks="3"/>
        </Sequence>
     </BehaviorTree>

 </root>
 )";


//  R"(

//  <root main_tree_to_execute = "MainTree" >

//      <BehaviorTree ID="MainTree">
//         <Sequence name="root_sequence">
//             <CheckBattery   name="battery_ok"/>
//             <OpenGripper    name="open_gripper"/>
//             <ApproachObject name="approach_object"/>
//             <CloseGripper   name="close_gripper"/>
//         </Sequence>
//      </BehaviorTree>

//  </root>
//  )";

// clang-format on

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  // We use the BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;

  /* There are two ways to register nodes:
    *    - statically, i.e. registering all the nodes one by one.
    *    - dynamically, loading the TreeNodes from a shared library (plugin).
    * */


  // Note: the name used to register should be the same used in the XML.
  // Note that the same operations could be done using DummyNodes::RegisterNodes(factory)

  // using namespace DummyNodes;

  // The recommended way to create a Node is through inheritance.
  // Even if it requires more boilerplate, it allows you to use more functionalities
  // like ports (we will discuss this in future tutorials).

  // factory.registerNodeType<ApproachObject>("ApproachObject");

  factory.registerNodeType<TickCount>("TickCount");

  // Registering a SimpleActionNode using a function pointer.
  // you may also use C++11 lambdas instead of std::bind
  // factory.registerSimpleCondition("CheckBattery", [&](TreeNode&) { return CheckBattery(); });

  //You can also create SimpleActionNodes using methods of a class
  // GripperInterface gripper;
  // factory.registerSimpleAction("OpenGripper", [&](TreeNode&){ return gripper.open(); } );
  // factory.registerSimpleAction("CloseGripper", [&](TreeNode&){ return gripper.close(); } );

// #else
  // Load dynamically a plugin and register the TreeNodes it contains
  // it automated the registering step.
  // factory.registerFromPlugin("./libdummy_nodes_dyn.so");


  // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
  // The currently supported format is XML.
  // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
  auto tree = factory.createTreeFromText(xml_text);

  // To "execute" a Tree you need to "tick" it.
  // The tick is propagated to the children based on the logic of the tree.
  // In this case, the entire sequence is executed, because all the children
  // of the Sequence return SUCCESS.


   // Monitor the execution using a ZMQ Publisher
  BT::PublisherZMQ publisher_zmq(tree);
  


  while (rclcpp::ok())
  {
      auto status = tree.rootNode()->executeTick();
      switch(status)
      {
          case BT::NodeStatus::RUNNING:
              std::cout << "Tree is RUNNING" << std::endl;
              break;
          case BT::NodeStatus::SUCCESS:
              std::cout << "Tree is SUCCESS" << std::endl;
              break;
          case BT::NodeStatus::FAILURE:
              std::cout << "Tree is FAILURE" << std::endl;
              break;
          default:
              break;
      }
      
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  rclcpp::shutdown();
  
  // tree.tickRootWhileRunning();

  return 0;
}

/* Expected output:
*
       [ Battery: OK ]
       GripperInterface::open
       ApproachObject: approach_object
       GripperInterface::close
*/