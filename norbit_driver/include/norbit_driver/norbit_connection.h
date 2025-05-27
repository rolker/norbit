#ifndef NORBIT_CONNECTION_H
#define NORBIT_CONNECTION_H

#include <atomic>
#include <boost/asio.hpp>
#include <chrono>
#include <deque>
#include <future>
#include <iostream>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "norbit_interfaces/msg/cmd_resp.hpp"
#include "norbit_interfaces/srv/norbit_cmd.hpp"
#include "norbit_interfaces/srv/set_power.hpp"
#include "norbit_types/message.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "conversions.h"


struct ConnectionParams {
  std::string ip;
  int bathy_port;
  int water_column_port;
  int cmd_port;
  std::string sensor_frame = "norbit";
  bool publish_pointcloud = true;
  bool publish_bathymetry = false;
  bool publish_detections = false;
  bool publish_ranges = false;
  bool publish_norbit_watercolumn = false;
  bool publish_watercolumn = false; //true;
  double cmd_timeout = 0.5;
  std::vector<std::string> startup_settings={"set_power 0"}; // default to not pinging
  std::vector<std::string> shutdown_settings={"set_power 0"};
};

class NorbitConnection: public rclcpp_lifecycle::LifecycleNode {
public:
  explicit NorbitConnection(const std::string & node_name);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

  bool updateParams();
  void setupPubSub();
  bool openConnections();
  void closeConnections();
  void initializeSonarParams();
  norbit_interfaces::msg::CmdResp sendCmd(std::string const &cmd, const std::string &val);
  void listenForCmd();
  void receiveCmd(const boost::system::error_code &err);

  void receiveWC();
  void wcHandler(const boost::system::error_code &error, // Result of operation.
             std::size_t bytes_transferred // Number of bytes received.
  );

  void receiveBathy();
  void bathyHandler(const boost::system::error_code &error, // Result of operation.
             std::size_t bytes_transferred // Number of bytes received.
  );

  void processHdrMsg(boost::asio::ip::tcp::socket &sock, boost::array<char, sizeof(norbit_interfaces::msg::CommonHeader)> & hdr);

  // norbit TCP callbacks
  void bathyCallback(norbit_types::BathymetricData data);
  void wcCallback(norbit_types::WaterColumnData data);

  // ROS callbacks
  void disconnectTimerCallback();

  void norbitCmdCallback(
    const std::shared_ptr<norbit_interfaces::srv::NorbitCmd::Request> request,
    std::shared_ptr<norbit_interfaces::srv::NorbitCmd::Response> response
  );

  void setPowerCallback(
    const std::shared_ptr<norbit_interfaces::srv::SetPower::Request> request,
    std::shared_ptr<norbit_interfaces::srv::SetPower::Response> response
  );

  // operations
  void spin_once();

protected:
  struct {
    std::unique_ptr<boost::asio::ip::tcp::socket> bathymetric;
    std::unique_ptr<boost::asio::ip::tcp::socket> water_column;
    std::unique_ptr<boost::asio::ip::tcp::socket> cmd;
  } sockets_;
  struct{
    boost::array<char, sizeof(norbit_interfaces::msg::CommonHeader)> bathymetric;
    boost::array<char, sizeof(norbit_interfaces::msg::CommonHeader)> water_column;
  } hdr_buff_;
  boost::asio::io_service io_service_;
  boost::asio::streambuf cmd_resp_buffer_;
  ConnectionParams params_;

  rclcpp::Service<norbit_interfaces::srv::NorbitCmd>::SharedPtr norbit_cmd_srv_;
  rclcpp::Service<norbit_interfaces::srv::SetPower>::SharedPtr set_power_srv_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<marine_acoustic_msgs::msg::SonarDetections>::SharedPtr detections_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<marine_acoustic_msgs::msg::SonarRanges>::SharedPtr ranges_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<norbit_interfaces::msg::BathymetricStamped>::SharedPtr bathymetry_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<norbit_interfaces::msg::WaterColumnStamped>::SharedPtr norbit_watercolumn_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<marine_acoustic_msgs::msg::RawSonarImage>::SharedPtr watercolumn_publisher_;
  std::deque<std::string> cmd_resp_queue_;
  rclcpp::TimerBase::SharedPtr spin_timer_;
  rclcpp::TimerBase::SharedPtr disconnect_timer_;
};

#endif // NORBIT_CONNECTION_H
