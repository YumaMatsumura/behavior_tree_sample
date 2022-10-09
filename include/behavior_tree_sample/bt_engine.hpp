#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

class BtEngine : public rclcpp::Node
{
public:
  BtEngine(
    const std::string& name_space = "",
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  
private:
  void timerCallback();
  void loadPlugins();
  void loadTree();
  
  rclcpp::TimerBase::SharedPtr timer_;
  
  BT::BehaviorTreeFactory factory_;
  std::shared_ptr<BT::Tree> tree_;
  
  std::vector<std::string> plugin_lib_names_;
  std::string bt_file_path_;
};
