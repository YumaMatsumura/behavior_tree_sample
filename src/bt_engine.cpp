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
  
  
  // Get ros2 params
  std::string xml_package_path;
  std::string xml_filename;
  this->get_parameter("plugin_lib_names", plugin_lib_names_);
  this->get_parameter("xml_package_path", xml_package_path);
  this->get_parameter("xml_filename", xml_filename);
  
  bt_file_path_ = xml_package_path + "/" + xml_filename;
  
  loadPlugins();
  loadTree();
  
  // Create timer
  timer_ = this->create_wall_timer(500ms, std::bind(&BtEngine::timerCallback, this));
}

void BtEngine::timerCallback()
{
  tree_->tickRoot();
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
