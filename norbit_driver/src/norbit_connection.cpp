#include "norbit_driver/norbit_connection.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/bind.hpp>
#include <boost/exception/diagnostic_information.hpp>

using namespace std::chrono_literals;

NorbitConnection::NorbitConnection(const std::string & node_name)
 : rclcpp_lifecycle::LifecycleNode(node_name)
{
}

NorbitConnection::CallbackReturn NorbitConnection::on_configure(const rclcpp_lifecycle::State &state)
{
  if(!updateParams()){
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(get_logger(), "Norbit MBES ip address : " << params_.ip);
  RCLCPP_INFO_STREAM(get_logger(), "  Bathymetry port      : " << params_.bathy_port);

  setupPubSub();

  return LifecycleNode::on_configure(state);
}

NorbitConnection::CallbackReturn NorbitConnection::on_activate(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Connecting to sonar");
  if(!openConnections()){
    RCLCPP_ERROR(get_logger(), "Unable to connect to sonar");
    return CallbackReturn::ERROR;
  }

  return LifecycleNode::on_activate(state);
}

std::pair<std::string, std::string> splitCmd(const std::string &cmd) {
  std::pair<std::string, std::string> out;
  std::istringstream iss(cmd);
  iss >> out.first;
  out.second = iss.str().substr(out.first.length());
  return out;
}


NorbitConnection::CallbackReturn NorbitConnection::on_deactivate(const rclcpp_lifecycle::State &state)
{
  for (auto param : params_.shutdown_settings) {
    auto split_param = splitCmd(param);
    sendCmd(split_param.first, split_param.second);
  }

  closeConnections();

  return LifecycleNode::on_deactivate(state);
  
}

NorbitConnection::CallbackReturn NorbitConnection::on_cleanup(const rclcpp_lifecycle::State &state)
{
  closeConnections();
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


bool NorbitConnection::updateParams() {
  if(!has_parameter("sensor_frame"))
    declare_parameter("sensor_frame", "norbit");
  params_.sensor_frame = get_parameter("sensor_frame").as_string();

  if(!has_parameter("ip"))
    declare_parameter("ip", "");
  params_.ip = get_parameter("ip").as_string();
  if(params_.ip == ""){
    RCLCPP_ERROR(get_logger(), "No IP address provided for the sonar. Set the 'ip' parameter");
    return false;
  }

  if(!has_parameter("bathy_port"))
    declare_parameter("bathy_port", 2210);
  params_.bathy_port = get_parameter("bathy_port").as_int();

  if(!has_parameter("water_column_port"))
    declare_parameter("water_column_port", 2211);
  params_.water_column_port = get_parameter("water_column_port").as_int();

  if(!has_parameter("cmd_port"))
    declare_parameter("cmd_port", 2209);
  params_.cmd_port = get_parameter("cmd_port").as_int();


  // detections stuff
  if(!has_parameter("publish_pointcloud"))
    declare_parameter("publish_pointcloud", params_.publish_pointcloud);
  params_.publish_pointcloud = get_parameter("publish_pointcloud").as_bool();

  if(!has_parameter("publish_bathymetry"))
    declare_parameter("publish_bathymetry", params_.publish_bathymetry);
  params_.publish_bathymetry = get_parameter("publish_bathymetry").as_bool();
  if(!has_parameter("publish_detections"))
    declare_parameter("publish_detections", params_.publish_detections);
  params_.publish_detections = get_parameter("publish_detections").as_bool();
  if(!has_parameter("publish_ranges"))
    declare_parameter("publish_ranges", params_.publish_ranges);
  params_.publish_ranges = get_parameter("publish_ranges").as_bool();

  // Watercolumn stuff
 
  if(!has_parameter("publish_norbit_watercolumn"))
    declare_parameter("publish_norbit_watercolumn", params_.publish_norbit_watercolumn);
  params_.publish_norbit_watercolumn = get_parameter("publish_norbit_watercolumn").as_bool();

  if(!has_parameter("publish_watercolumn"))
    declare_parameter("publish_watercolumn", params_.publish_watercolumn);
  params_.publish_watercolumn = get_parameter("publish_watercolumn").as_bool();
 
  if(!has_parameter("cmd_timeout"))
    declare_parameter("cmd_timeout", params_.cmd_timeout);
  params_.cmd_timeout = get_parameter("cmd_timeout").as_double();

  if(!has_parameter("startup_settings"))
    declare_parameter("startup_settings", params_.startup_settings);
  params_.startup_settings = get_parameter("startup_settings").as_string_array();

  if(!has_parameter("shutdown_settings"))
    declare_parameter("shutdown_settings", params_.shutdown_settings);
  params_.shutdown_settings = get_parameter("shutdown_settings").as_string_array();

  return true;
}

void NorbitConnection::setupPubSub() {

  // publishers
  if (params_.publish_pointcloud){
    pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("soundings", 1);
  }

  if (params_.publish_detections){
    detections_publisher_ = create_publisher<marine_acoustic_msgs::msg::SonarDetections>("detections", 1);
  } 

  if (params_.publish_ranges){
    ranges_publisher_ = create_publisher<marine_acoustic_msgs::msg::SonarRanges>("ranges", 1);
  }

  if(params_.publish_bathymetry){
    bathymetry_publisher_ = create_publisher<norbit_interfaces::msg::BathymetricStamped>("bathymetry", 1);
  }

  if(params_.publish_norbit_watercolumn){
    norbit_watercolumn_publisher_ = create_publisher<norbit_interfaces::msg::WaterColumnStamped>("norbit_watercolumn", 1);
  }

  if(params_.publish_watercolumn){
    watercolumn_publisher_ = create_publisher<marine_acoustic_msgs::msg::RawSonarImage>("watercolumn", 1);
  }


  // SRVs

  norbit_cmd_srv_ = create_service<norbit_interfaces::srv::NorbitCmd>(
      "norbit_cmd",
      std::bind(
        &NorbitConnection::norbitCmdCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
  );

  set_power_srv_ = create_service<norbit_interfaces::srv::SetPower>(
      "set_power",
      std::bind(
        &NorbitConnection::setPowerCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
  );


  // timers

  spin_timer_ = create_wall_timer(
    100ms,
    std::bind(&NorbitConnection::spin_once, this)
  );
}

bool NorbitConnection::openConnections() {
  try {
    io_service_.restart();
    sockets_.bathymetric = std::unique_ptr<boost::asio::ip::tcp::socket>(
        new boost::asio::ip::tcp::socket(io_service_));

    sockets_.bathymetric->async_connect(
      boost::asio::ip::tcp::endpoint(
        boost::asio::ip::address::from_string(params_.ip),
        params_.bathy_port),
        [this](const boost::system::error_code &ec) {
          if (ec) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("norbit_driver"), "Error connecting to bathymetric socket: " << ec.message());
          }
          else
          {
            this->receiveBathy();
          }
        }
    );

    if(params_.publish_norbit_watercolumn || params_.publish_watercolumn){
      sockets_.water_column = std::unique_ptr<boost::asio::ip::tcp::socket>(
          new boost::asio::ip::tcp::socket(io_service_));

      sockets_.water_column->async_connect(
        boost::asio::ip::tcp::endpoint(
          boost::asio::ip::address::from_string(params_.ip),
          params_.water_column_port
        ),
        [this](const boost::system::error_code &ec) {
          if (ec) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("norbit_driver"), "Error connecting to water column socket: " << ec.message());
          }
          else
          {
            this->receiveWC();
          }
        }
      );
    }
    
    sockets_.cmd = std::unique_ptr<boost::asio::ip::tcp::socket>(
        new boost::asio::ip::tcp::socket(io_service_));

    sockets_.cmd->async_connect(
      boost::asio::ip::tcp::endpoint(
        boost::asio::ip::address::from_string(params_.ip),
        params_.cmd_port
      ),
      [this](const boost::system::error_code &ec) {
        if (ec) {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("norbit_driver"), "Error connecting to command socket: " << ec.message());
        }
        else
        {
          this->initializeSonarParams();
        }
      }
    );

    disconnect_timer_ = create_wall_timer(1s, std::bind(&NorbitConnection::disconnectTimerCallback, this));
        
    return true;
  }
  catch (const boost::exception& ex) {
      std::string info = boost::diagnostic_information(ex);
      RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 10.0,
          "Unable to connect to sonar: " << info);
      return false;
  }

}

void NorbitConnection::closeConnections() {
  if(sockets_.bathymetric)
  {
    sockets_.bathymetric->close();
    sockets_.bathymetric.reset();
  }
  if(sockets_.water_column)
  {
    sockets_.water_column->close();
    sockets_.water_column.reset();
  }
  if(sockets_.cmd)
  {
    sockets_.cmd->close();
    sockets_.cmd.reset();
  }
  disconnect_timer_.reset();
}

void removeSubstrs(std::string &s, const std::string p) {
  std::string::size_type n = p.length();
  for (std::string::size_type i = s.find(p); i != std::string::npos;
       i = s.find(p))
    s.erase(i, n);
}

void NorbitConnection::initializeSonarParams(){
  for (auto param : params_.startup_settings) {
    auto split_param = splitCmd(param);
    while(!sendCmd(split_param.first, split_param.second).ack){
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>( std::chrono::duration<double>(params_.cmd_timeout)));
    }
  }
}

norbit_interfaces::msg::CmdResp NorbitConnection::sendCmd(
  const std::string &cmd,
  const std::string &val
) {

  norbit_interfaces::msg::CmdResp out;
  out.success = false;
  out.ack = false;
  out.resp = "";

  std::string message = cmd + " " + val + "\n";
  std::string key = cmd;

  // some of the norbit reponses don't echo back set_<cmd> so we need to strip
  // it
  removeSubstrs(key, "set_");
  removeSubstrs(key, " ");

  RCLCPP_INFO_STREAM(get_logger(), "command message sent: " << message);
  if(!sockets_.cmd || !sockets_.cmd->is_open()){
    RCLCPP_ERROR(get_logger(), "Command socket not open");
    return out;
  }

  boost::asio::async_write(
    *sockets_.cmd,
    boost::asio::buffer(message),
    boost::bind(
      &NorbitConnection::listenForCmd,
      this
    )
  );
  

  auto start_time = std::chrono::system_clock::now();
  auto timeout = start_time + std::chrono::duration<double>(params_.cmd_timeout);
  bool running = true;
  do {
    spin_once();
    if (cmd_resp_queue_.size() > 0) {
      out.resp = cmd_resp_queue_.front();
      RCLCPP_INFO_STREAM(get_logger(), "Received response: " << out.resp);
      if (cmd_resp_queue_.front().find(key) != std::string::npos) {
        RCLCPP_INFO_STREAM(get_logger(), "ACK Received: " << cmd_resp_queue_.front());
        cmd_resp_queue_.pop_front();
        out.success = true;
        out.ack = true;
        running = false;
      } else {
        cmd_resp_queue_.pop_front();
      }
    } else {
      if (std::chrono::system_clock::now() > timeout) {

        if (out.resp != "") {
          RCLCPP_ERROR_STREAM(get_logger(), "[" << get_name() <<  "] received bad ACK: " << out.resp);
          out.ack = true;
        } else {
          RCLCPP_ERROR_STREAM(get_logger(), "[" << get_name() <<  "] TIMEOUT -- no ACK received");
          out.ack = false;
        }
        running = false;
      }
    }
  } while (running);

  return out;
}

void NorbitConnection::listenForCmd() {

  boost::asio::async_read_until(*sockets_.cmd, cmd_resp_buffer_, "\r\n",
                                boost::bind(&NorbitConnection::receiveCmd, this,
                                            boost::asio::placeholders::error));
}

void NorbitConnection::receiveCmd(const boost::system::error_code &err) {
  std::string line;
  std::istream is(&cmd_resp_buffer_);
  std::getline(is, line);
  cmd_resp_queue_.push_back(line);
  listenForCmd();
  return;
}

void NorbitConnection::receiveWC() {
  hdr_buff_.water_column.assign(0);

  sockets_.water_column->async_receive(
      boost::asio::buffer(hdr_buff_.water_column), 0,
      boost::bind(&NorbitConnection::wcHandler, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void NorbitConnection::wcHandler(const boost::system::error_code &error,
                                  std::size_t bytes_transferred) {

  processHdrMsg(*sockets_.water_column,hdr_buff_.water_column);
  receiveWC();
  return;
}

void NorbitConnection::receiveBathy() {
  hdr_buff_.bathymetric.assign(0);
  sockets_.bathymetric->async_receive(
      boost::asio::buffer(hdr_buff_.bathymetric), 0,
      boost::bind(&NorbitConnection::bathyHandler, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void NorbitConnection::bathyHandler(const boost::system::error_code &error,
                                  std::size_t bytes_transferred) {

  processHdrMsg(*sockets_.bathymetric,hdr_buff_.bathymetric);
  receiveBathy();
  return;
}

void NorbitConnection::processHdrMsg(boost::asio::ip::tcp::socket & sock, boost::array<char, sizeof(norbit_interfaces::msg::CommonHeader)> &hdr){
  if(disconnect_timer_)
    disconnect_timer_->reset();
  try {
    norbit_types::Message msg;
    if (msg.fromBoostArray(hdr)) {
      const unsigned int dataSize = msg.commonHeader().size - sizeof(norbit_interfaces::msg::CommonHeader);
      std::shared_ptr<char> dataPtr;
      dataPtr.reset(new char[dataSize]);
      size_t bytesRead =read(sock,boost::asio::buffer(dataPtr.get(), dataSize));

      RCLCPP_INFO_STREAM(get_logger(), "Received " << bytesRead << " bytes of data since the common header");

      if(msg.setBits(dataPtr)){
        if (msg.commonHeader().type == norbit_types::bathymetric) {
          bathyCallback(msg.getBathy());
        }
        if (msg.commonHeader().type == norbit_types::watercolum){
           wcCallback(msg.getWC());
        }
      }
      else RCLCPP_WARN(get_logger(), "Watercolumn Message failed CRC check:  Ignoring");

    }else{
      RCLCPP_ERROR_STREAM(get_logger(), "Header: " << to_yaml(msg.commonHeader()));
      if(msg.commonHeader().preable==norbit_interfaces::msg::CommonHeader::NORBIT_PREAMBLE_KEY)
        RCLCPP_WARN(get_logger(), "Invalid header preamble detected");
    }

  } catch (...) {
    RCLCPP_ERROR(get_logger(), "An unhandled exception occurred in NorbitConnection::recHandler()");
    throw;
  }
}



void NorbitConnection::bathyCallback(norbit_types::BathymetricData data) {

  RCLCPP_INFO_STREAM(get_logger(), "Received bathymetric data: "
      <<  to_yaml(data.bathymetricHeader()));

  RCLCPP_INFO_STREAM(get_logger(), "Number of bytes we should have: "
      << data.bathymetricHeader().n*sizeof(norbit_interfaces::msg::BathymetricPoint)+sizeof(norbit_interfaces::msg::BathymetricHeader));

  pcl::PointCloud<pcl::PointXYZI>::Ptr detections(
      new pcl::PointCloud<pcl::PointXYZI>);
  detections->header.frame_id = params_.sensor_frame;
  rclcpp::Time stamp(data.bathymetricHeader().time*1000000000);
  if (params_.publish_pointcloud){
    for (size_t i = 0; i < data.bathymetricHeader().n; i++) {
      if (data.data(i).sample_number > 1) {
        float range = float(data.data(i).sample_number) *
                      data.bathymetricHeader().sound_velocity /
                      (2.0 * data.bathymetricHeader().sample_rate);
        if (i<10)
          RCLCPP_INFO_STREAM(get_logger(), i << " sounding: " << to_yaml(data.data(i)));
        pcl::PointXYZI p;
        p.x = range * sinf(data.bathymetricHeader().tx_angle);
        p.y = range * sinf(data.data(i).angle);
        p.z = range * cosf(data.data(i).angle);
        p.intensity = float(data.data(i).intensity) / 1e9f;
        if ( data.data(i).quality_flag == 3) {
          detections->push_back(p);
        }
      }
    }
    pcl_conversions::toPCL(stamp, detections->header.stamp);
    sensor_msgs::msg::PointCloud2 pc2;
    pcl::toROSMsg(*detections, pc2);
    pc2.header.frame_id = params_.sensor_frame;
    pc2.header.stamp = stamp;
    pointcloud_publisher_->publish(pc2);
  }

  auto bathy_msg = data.getRosMsg(params_.sensor_frame);
  if (params_.publish_bathymetry) {
    bathymetry_publisher_->publish(bathy_msg);
  }

  if (params_.publish_detections) {
    marine_acoustic_msgs::msg::SonarDetections detections_msg;
    norbit::conversions::bathymetric2SonarDetections(bathy_msg, detections_msg);
    detections_publisher_->publish(detections_msg);
  }

  if (params_.publish_ranges) {
    marine_acoustic_msgs::msg::SonarRanges ranges_msg;
    norbit::conversions::bathymetric2SonarRanges(bathy_msg, ranges_msg);
    ranges_publisher_->publish(ranges_msg);
  }

  return;
}

void NorbitConnection::wcCallback(norbit_types::WaterColumnData data){
  auto norb_wc_msg = data.getRosMsg(params_.sensor_frame);
  if(params_.publish_norbit_watercolumn)
    norbit_watercolumn_publisher_->publish(norb_wc_msg);

  if(params_.publish_watercolumn){
    auto hydro_wc_msg = std::make_shared<marine_acoustic_msgs::msg::RawSonarImage>();
    norbit::conversions::norbitWC2RawSonarImage(norb_wc_msg, *hydro_wc_msg);
    watercolumn_publisher_->publish(*hydro_wc_msg);
  }
}

void NorbitConnection::disconnectTimerCallback(){
  RCLCPP_INFO(get_logger(), "No Messages received for a while.   Checking Connections");
  if(!sendCmd("set_power", "").ack){
    RCLCPP_ERROR(get_logger(), "Sonar disconnected: restarting connections");
    closeConnections();
    openConnections();
  }
  return;
}

void NorbitConnection::norbitCmdCallback(
  const std::shared_ptr<norbit_interfaces::srv::NorbitCmd::Request> request,
  std::shared_ptr<norbit_interfaces::srv::NorbitCmd::Response> response) {
  response->resp = sendCmd(request->cmd, request->val);
}

void NorbitConnection::setPowerCallback(
  const std::shared_ptr<norbit_interfaces::srv::SetPower::Request> request,
  std::shared_ptr<norbit_interfaces::srv::SetPower::Response> response) {
  response->resp = sendCmd("set_power", std::to_string(request->on));
}

void NorbitConnection::spin_once() {
  io_service_.run_for(100ms);
}





