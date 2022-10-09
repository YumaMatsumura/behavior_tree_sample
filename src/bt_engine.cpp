#include "behavior_tree_sample/bt_engine.hpp"

BtEngine::BtEngine(
  const std::string& name_space,
  const rclcpp::NodeOptions& options
): Node("bt_engine", name_space, options)
{
  using namespace std::chrono_literals;

  const std::vector<std::string> plugin_libs = {
    "hello_world_action_bt_node"
  };
  
  const std::string package_path = ament_index_cpp::get_package_share_directory("behavior_tree_sample") + "/behavior_trees";
  
  // Set ros2 params
  this->declare_parameter<std::vector<std::string>>("plugin_lib_names", plugin_libs);
  this->declare_parameter<std::string>("xml_package_path", package_path);
  this->declare_parameter<std::string>("xml_filename", "hello_world.xml");
  this->declare_parameter<uint16_t>("publisher_port", 1666);
  this->declare_parameter<uint16_t>("server_port", 1667);
  this->declare_parameter<uint16_t>("max_msg_per_second", 25);
  this->declare_parameter<bool>("use_groot_monitor", true);
  
  
  // Get ros2 params
  std::string xml_package_path;
  std::string xml_filename;
  bool use_groot_monitor;
  this->get_parameter("plugin_lib_names", plugin_lib_names_);
  this->get_parameter("xml_package_path", xml_package_path);
  this->get_parameter("xml_filename", xml_filename);
  this->get_parameter("publisher_port", publisher_port_);
  this->get_parameter("server_port", server_port_);
  this->get_parameter("max_msg_per_second", max_msg_per_second_);
  this->get_parameter("use_groot_monitor", use_groot_monitor);
  
  bt_file_path_ = xml_package_path + "/" + xml_filename;
  
  loadPlugins();
  loadTree();
  if(use_groot_monitor)
  {
    publishToGrootMonitor();
  }
  
  // Create timer
  timer_ = this->create_wall_timer(500ms, std::bind(&BtEngine::timerCallback, this));
}

void BtEngine::timerCallback()
{
  tree_->tickRoot();
}

void BtEngine::publishToGrootMonitor()
{
  groot_monitor_ = std::make_unique<BT::PublisherZMQ>(
    *tree_, max_msg_per_second_, publisher_port_, server_port_
  );
}

void BtEngine::loadPlugins()
{
  BT::SharedLibrary loader;
  for(const auto & p : plugin_lib_names_)
  {
    RCLCPP_INFO(this->get_logger(), "Loading plugin: %s", loader.getOSName(p).c_str());
    factory_.registerFromPlugin(loader.getOSName(p));
  }
}

void BtEngine::loadTree()
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", std::make_shared<rclcpp::Node>("bt_node"));
  tree_ = std::make_shared<BT::Tree>(factory_.createTreeFromFile(bt_file_path_, blackboard));
}
