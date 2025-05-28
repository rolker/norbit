#ifndef NORBIT_DRIVER_NORBIT_NODE_H
#define NORBIT_DRIVER_NORBIT_NODE_H

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "norbit_interfaces/srv/norbit_cmd.hpp"
#include "norbit_interfaces/srv/set_power.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "norbit_driver/norbit_connection.h"

class NorbitNode : public rclcpp_lifecycle::LifecycleNode
{
public:

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


  explicit NorbitNode(const std::string & node_name);

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

  void pointcloudCallback(const norbit_types::BathymetricData &data);
  void rangesCallback(const norbit_types::BathymetricData &data);
  void bathymetryCallback(const norbit_types::BathymetricData &data);
  void detectionsCallback(const norbit_types::BathymetricData &data);

  void norbitWatercolumnCallback(const norbit_types::WaterColumnData &data);
  void watercolumnCallback(const norbit_types::WaterColumnData &data);

private:
  void norbitCmdCallback(
    const std::shared_ptr<norbit_interfaces::srv::NorbitCmd::Request> request,
    std::shared_ptr<norbit_interfaces::srv::NorbitCmd::Response> response
  );

  void setPowerCallback(
    const std::shared_ptr<norbit_interfaces::srv::SetPower::Request> request,
    std::shared_ptr<norbit_interfaces::srv::SetPower::Response> response
  );

  void spin_once();

  std::string sonar_ip;
  int command_port = 2209;
  int bathymetry_port = 2210;
  int water_column_port = 2211;
  
  std::string sensor_frame = "norbit";
  bool publish_pointcloud = true;
  bool publish_bathymetry = false;
  bool publish_detections = false;
  bool publish_ranges = false;
  bool publish_norbit_watercolumn = false;
  bool publish_watercolumn = false;
  double command_timeout = 0.5;
  std::vector<std::string> startup_settings={"set_power 0"}; // default to not pinging
  std::vector<std::string> shutdown_settings={"set_power 0"};

  std::shared_ptr<NorbitConnection> connection_;

  rclcpp::Service<norbit_interfaces::srv::NorbitCmd>::SharedPtr norbit_cmd_srv_;
  rclcpp::Service<norbit_interfaces::srv::SetPower>::SharedPtr set_power_srv_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<marine_acoustic_msgs::msg::SonarDetections>::SharedPtr detections_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<marine_acoustic_msgs::msg::SonarRanges>::SharedPtr ranges_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<norbit_interfaces::msg::BathymetricStamped>::SharedPtr bathymetry_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<norbit_interfaces::msg::WaterColumnStamped>::SharedPtr norbit_watercolumn_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<marine_acoustic_msgs::msg::RawSonarImage>::SharedPtr watercolumn_publisher_;

  rclcpp::TimerBase::SharedPtr spin_timer_;


};

#endif

