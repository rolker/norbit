#include "norbit_driver/norbit_node.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

NorbitNode::NorbitNode(const std::string & node_name)
  : rclcpp_lifecycle::LifecycleNode(node_name)
{
}

NorbitNode::CallbackReturn NorbitNode::on_configure(const rclcpp_lifecycle::State &state)
{
  if(!has_parameter("sensor_frame"))
    declare_parameter("sensor_frame", sensor_frame);
  sensor_frame = get_parameter("sensor_frame").as_string();

  if(!has_parameter("sonar_ip"))
    declare_parameter("sonar_ip", "");
  sonar_ip = get_parameter("sonar_ip").as_string();

  if(sonar_ip == ""){
    RCLCPP_ERROR(get_logger(), "No IP address provided for the sonar. Set the 'sonar_ip' parameter");
    return CallbackReturn::ERROR;
  }

  if(!has_parameter("bathymetry_port"))
    declare_parameter("bathymetry_port", bathymetry_port);
  bathymetry_port = get_parameter("bathymetry_port").as_int();

  if(!has_parameter("water_column_port"))
    declare_parameter("water_column_port", water_column_port);
  water_column_port = get_parameter("water_column_port").as_int();

  if(!has_parameter("command_port"))
    declare_parameter("command_port", command_port);
  command_port = get_parameter("command_port").as_int();


  // detections stuff
  if(!has_parameter("publish_pointcloud"))
    declare_parameter("publish_pointcloud", publish_pointcloud);
  publish_pointcloud = get_parameter("publish_pointcloud").as_bool();

  if(!has_parameter("publish_bathymetry"))
    declare_parameter("publish_bathymetry", publish_bathymetry);
  publish_bathymetry = get_parameter("publish_bathymetry").as_bool();
  
  if(!has_parameter("publish_detections"))
    declare_parameter("publish_detections", publish_detections);
  publish_detections = get_parameter("publish_detections").as_bool();
  
  if(!has_parameter("publish_ranges"))
    declare_parameter("publish_ranges", publish_ranges);
  publish_ranges = get_parameter("publish_ranges").as_bool();

  // Watercolumn stuff
 
  if(!has_parameter("publish_norbit_watercolumn"))
    declare_parameter("publish_norbit_watercolumn", publish_norbit_watercolumn);
  publish_norbit_watercolumn = get_parameter("publish_norbit_watercolumn").as_bool();

  if(!has_parameter("publish_watercolumn"))
    declare_parameter("publish_watercolumn", publish_watercolumn);
  publish_watercolumn = get_parameter("publish_watercolumn").as_bool();
 
  if(!has_parameter("command_timeout"))
    declare_parameter("command_timeout", command_timeout);
  command_timeout = get_parameter("command_timeout").as_double();

  if(!has_parameter("startup_settings"))
    declare_parameter("startup_settings", startup_settings);
  startup_settings = get_parameter("startup_settings").as_string_array();

  if(!has_parameter("shutdown_settings"))
    declare_parameter("shutdown_settings", shutdown_settings);
  shutdown_settings = get_parameter("shutdown_settings").as_string_array();

  RCLCPP_INFO_STREAM(get_logger(), "Norbit MBES ip address : " << sonar_ip);

  connection_ = std::make_shared<NorbitConnection>();

  NorbitConnection::ConnectionParams params;
  params.sonar_ip = sonar_ip;
  params.bathymetry_port = bathymetry_port;
  params.water_column_port = water_column_port;
  params.command_port = command_port;
  params.cmd_timeout = command_timeout;
  connection_->setParameters(params);

  if (publish_pointcloud){
    pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("soundings", 1);
    connection_->addCallback(
      std::bind(&NorbitNode::pointcloudCallback, this, std::placeholders::_1)
    );
  }

  if (publish_detections){
    detections_publisher_ = create_publisher<marine_acoustic_msgs::msg::SonarDetections>("detections", 1);
    connection_->addCallback(
      std::bind(&NorbitNode::detectionsCallback, this, std::placeholders::_1)
    );
  } 

  if (publish_ranges){
    ranges_publisher_ = create_publisher<marine_acoustic_msgs::msg::SonarRanges>("ranges", 1);
    connection_->addCallback(
      std::bind(&NorbitNode::rangesCallback, this, std::placeholders::_1)
    );
  }

  if(publish_bathymetry){
    bathymetry_publisher_ = create_publisher<norbit_interfaces::msg::BathymetricStamped>("bathymetry", 1);
    connection_->addCallback(
      std::bind(&NorbitNode::bathymetryCallback, this, std::placeholders::_1)
    );
  }

  if(publish_norbit_watercolumn){
    norbit_watercolumn_publisher_ = create_publisher<norbit_interfaces::msg::WaterColumnStamped>("norbit_watercolumn", 1);
    connection_->addCallback(
      std::bind(&NorbitNode::norbitWatercolumnCallback, this, std::placeholders::_1)
    );
  }

  if(publish_watercolumn){
    watercolumn_publisher_ = create_publisher<marine_acoustic_msgs::msg::RawSonarImage>("watercolumn", 1);
    connection_->addCallback(
      std::bind(&NorbitNode::watercolumnCallback, this, std::placeholders::_1)
    );
  }


  // SRVs

  norbit_cmd_srv_ = create_service<norbit_interfaces::srv::NorbitCmd>(
      "norbit_cmd",
      std::bind(
        &NorbitNode::norbitCmdCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
  );

  set_power_srv_ = create_service<norbit_interfaces::srv::SetPower>(
      "set_power",
      std::bind(
        &NorbitNode::setPowerCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
  );


  // timers

  spin_timer_ = create_wall_timer(
    100ms,
    std::bind(&NorbitNode::spin_once, this)
  );

  return LifecycleNode::on_configure(state);
}

std::pair<std::string, std::string> splitCmd(const std::string &cmd)
{
  std::pair<std::string, std::string> out;
  std::istringstream iss(cmd);
  iss >> out.first;
  out.second = iss.str().substr(out.first.length());
  return out;
}

NorbitNode::CallbackReturn NorbitNode::on_activate(const rclcpp_lifecycle::State &state)
{

  RCLCPP_INFO(get_logger(), "Connecting to sonar");
  for(const auto& response: connection_->connect())
  {
    RCLCPP_INFO_STREAM(get_logger(), "Connection response: " << response);
  }

  for (auto param : startup_settings) {
    RCLCPP_INFO_STREAM(get_logger(), "Sending startup command: " << param);
    auto split_param = splitCmd(param);
    auto response = connection_->sendCmd(split_param.first, split_param.second);
    RCLCPP_INFO_STREAM(get_logger(), "Response: " << to_yaml(response));
    while(!response.ack)
    {
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>( std::chrono::duration<double>(command_timeout)));
      response = connection_->sendCmd(split_param.first, split_param.second);
      RCLCPP_INFO_STREAM(get_logger(), "Response: " << to_yaml(response));
    }
  }

  return LifecycleNode::on_activate(state);
}



NorbitNode::CallbackReturn NorbitNode::on_deactivate(const rclcpp_lifecycle::State &state)
{
  for (auto param : shutdown_settings)
   {
    auto split_param = splitCmd(param);
    connection_->sendCmd(split_param.first, split_param.second);
  }

  connection_->closeConnections();
  connection_.reset();

  return LifecycleNode::on_deactivate(state);
  
}

NorbitNode::CallbackReturn NorbitNode::on_cleanup(const rclcpp_lifecycle::State &state)
{
  if(connection_)
    connection_->closeConnections();
  connection_.reset();
  pointcloud_publisher_.reset();
  detections_publisher_.reset();
  ranges_publisher_.reset();
  bathymetry_publisher_.reset();
  norbit_watercolumn_publisher_.reset();
  watercolumn_publisher_.reset();
  norbit_cmd_srv_.reset();
  set_power_srv_.reset();
  spin_timer_.reset();
  return LifecycleNode::on_cleanup(state);
}


void NorbitNode::norbitCmdCallback(
  const std::shared_ptr<norbit_interfaces::srv::NorbitCmd::Request> request,
  std::shared_ptr<norbit_interfaces::srv::NorbitCmd::Response> response
)
{
  response->resp = connection_->sendCmd(request->cmd, request->val);
  RCLCPP_INFO_STREAM(get_logger(), "Received command: " << request->cmd
      << " with value: " << request->val
      << " Response: " << to_yaml(response->resp));
}

void NorbitNode::setPowerCallback(
  const std::shared_ptr<norbit_interfaces::srv::SetPower::Request> request,
  std::shared_ptr<norbit_interfaces::srv::SetPower::Response> response
)
{
  response->resp = connection_->sendCmd("set_power", std::to_string(request->on));
  RCLCPP_INFO_STREAM(get_logger(), "Received set_power command with value: "
      << request->on << " Response: " << to_yaml(response->resp));
}

void NorbitNode::spin_once()
{
  if (connection_) {
    connection_->spin_once();
  }
}

void NorbitNode::pointcloudCallback(const norbit_types::BathymetricData &data)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr detections(
      new pcl::PointCloud<pcl::PointXYZI>);
  detections->header.frame_id = sensor_frame;
  rclcpp::Time stamp(data.bathymetricHeader().time*1000000000);
  for (size_t i = 0; i < data.bathymetricHeader().n; i++) {
    if (data.data(i).sample_number > 1) {
      float range = float(data.data(i).sample_number) *
                    data.bathymetricHeader().sound_velocity /
                    (2.0 * data.bathymetricHeader().sample_rate);
      pcl::PointXYZI p;
      p.x = range * sinf(data.bathymetricHeader().tx_angle);
      p.y = range * sinf(data.data(i).angle);
      p.z = range * cosf(data.data(i).angle);
      p.intensity = float(data.data(i).intensity) / 1e9f;
      if ( data.data(i).quality_flag == 3) {
        detections->push_back(p);
      }
    }
    pcl_conversions::toPCL(stamp, detections->header.stamp);
    sensor_msgs::msg::PointCloud2 pc2;
    pcl::toROSMsg(*detections, pc2);
    pc2.header.frame_id = sensor_frame;
    pc2.header.stamp = stamp;
    pointcloud_publisher_->publish(pc2);
  }

}


void NorbitNode::rangesCallback(const norbit_types::BathymetricData &data)
{
  auto bathy_msg = data.getRosMsg(sensor_frame);
  marine_acoustic_msgs::msg::SonarRanges ranges_msg;
  norbit::conversions::bathymetric2SonarRanges(bathy_msg, ranges_msg);
  ranges_publisher_->publish(ranges_msg);
}

void NorbitNode::bathymetryCallback(const norbit_types::BathymetricData &data)
{
  auto bathy_msg = data.getRosMsg(sensor_frame);
  bathymetry_publisher_->publish(bathy_msg);
}

void NorbitNode::detectionsCallback(const norbit_types::BathymetricData &data)
{
  auto bathy_msg = data.getRosMsg(sensor_frame);
  marine_acoustic_msgs::msg::SonarDetections detections_msg;
  norbit::conversions::bathymetric2SonarDetections(bathy_msg, detections_msg);
  detections_publisher_->publish(detections_msg);
}

void NorbitNode::norbitWatercolumnCallback(const norbit_types::WaterColumnData &data)
{
  auto norb_wc_msg = data.getRosMsg(sensor_frame);
  norbit_watercolumn_publisher_->publish(norb_wc_msg);
}

void NorbitNode::watercolumnCallback(const norbit_types::WaterColumnData &data)
{
  auto norb_wc_msg = data.getRosMsg(sensor_frame);
  auto hydro_wc_msg = std::make_shared<marine_acoustic_msgs::msg::RawSonarImage>();
  norbit::conversions::norbitWC2RawSonarImage(norb_wc_msg, *hydro_wc_msg);
  watercolumn_publisher_->publish(*hydro_wc_msg);

}

