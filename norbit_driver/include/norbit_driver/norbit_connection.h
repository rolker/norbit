#ifndef NORBIT_CONNECTION_H
#define NORBIT_CONNECTION_H

#include <atomic>
#include <boost/asio.hpp>
#include <chrono>
#include <deque>
#include <future>
#include <iostream>


#include "norbit_interfaces/msg/cmd_resp.hpp"
#include "norbit_types/message.h"

#include "conversions.h"



class NorbitConnection{
public:

  struct ConnectionParams {
    std::string sonar_ip;
    int bathymetry_port = 2210;
    int water_column_port = 2211;
    int command_port = 2209;
    double cmd_timeout = 0.5;
  };

  using BathymetryCallback = std::function<void(const norbit_types::BathymetricData&)>;
  using WaterColumnCallback = std::function<void(const norbit_types::WaterColumnData&)>;

  NorbitConnection();

  const ConnectionParams &parameters() const { return params_; }
  void setParameters(const ConnectionParams &params) { params_ = params; }

  void addCallback(BathymetryCallback cb);
  void addCallback(WaterColumnCallback cb);

  std::vector<std::string> connect();
  void closeConnections();

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
  } header_buffers_;

  boost::asio::io_service io_service_;
  boost::asio::streambuf cmd_resp_buffer_;
  ConnectionParams params_;

  std::deque<std::string> cmd_resp_queue_;

  std::vector<BathymetryCallback> bathymetry_callbacks_;
  std::vector<WaterColumnCallback> watercolumn_callbacks_;
  

};

#endif // NORBIT_CONNECTION_H
