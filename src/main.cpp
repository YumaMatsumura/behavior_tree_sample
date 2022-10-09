#include <csignal>

#include <rclcpp/rclcpp.hpp>
#include "behavior_tree_sample/bt_engine.hpp"

void sigint_handler(__attribute__((unused)) int signal_num)
{
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  signal(SIGINT, sigint_handler);
  
  auto node = std::make_shared<BtEngine>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
