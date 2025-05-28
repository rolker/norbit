#include "norbit_driver/norbit_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto norbit_node = std::make_shared<NorbitNode>("norbit_node");
  executor.add_node(norbit_node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
