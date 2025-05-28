#include "norbit_driver/norbit_connection.h"

#include <boost/bind.hpp>
#include <boost/exception/diagnostic_information.hpp>

using namespace std::chrono_literals;

NorbitConnection::NorbitConnection()
{
}

void NorbitConnection::addCallback(BathymetryCallback cb) {
  bathymetry_callbacks_.push_back(std::move(cb));

  if(!sockets_.bathymetric || !sockets_.bathymetric->is_open()) {
    sockets_.bathymetric = std::unique_ptr<boost::asio::ip::tcp::socket>(
    new boost::asio::ip::tcp::socket(io_service_));

    sockets_.bathymetric->async_connect(
    boost::asio::ip::tcp::endpoint(
      boost::asio::ip::address::from_string(params_.sonar_ip),
      params_.bathymetry_port),
      [this](const boost::system::error_code &ec) {
        if (ec) {
          throw std::runtime_error(
            "Error connecting to bathymetric socket: " + ec.message());
        }
        else
        {
          this->receiveBathy();
        }
      }
    );
  }
}

void NorbitConnection::addCallback(WaterColumnCallback cb) {
  watercolumn_callbacks_.push_back(std::move(cb));

  if(!sockets_.water_column || !sockets_.water_column->is_open()) {
    sockets_.water_column = std::unique_ptr<boost::asio::ip::tcp::socket>(
      new boost::asio::ip::tcp::socket(io_service_));

    sockets_.water_column->async_connect(
      boost::asio::ip::tcp::endpoint(
        boost::asio::ip::address::from_string(params_.sonar_ip),
        params_.water_column_port
      ),
      [this](const boost::system::error_code &ec) {
        if (ec) {
          throw std::runtime_error(
            "Error connecting to water column socket: " + ec.message());
        }
        else
        {
          this->receiveWC();
        }
      }
    );
  }
}

std::vector<std::string> NorbitConnection::connect() {
  io_service_.restart();

  sockets_.cmd = std::unique_ptr<boost::asio::ip::tcp::socket>(
    new boost::asio::ip::tcp::socket(io_service_));

  sockets_.cmd->async_connect(
    boost::asio::ip::tcp::endpoint(
      boost::asio::ip::address::from_string(params_.sonar_ip),
      params_.command_port
    ),
    [this](const boost::system::error_code &ec) {
      if (ec) {
        throw std::runtime_error(
          "Error connecting to command socket: " + ec.message());
      }
      this->listenForCmd();
    }
  );

  auto timeout = std::chrono::system_clock::now() + std::chrono::duration<double>(params_.cmd_timeout);

  std::vector<std::string> responses;

  do {
    spin_once();
    while(!cmd_resp_queue_.empty()) {
      responses.push_back(cmd_resp_queue_.front());
      cmd_resp_queue_.pop_front();
    }
  } while (std::chrono::system_clock::now() < timeout);


  return responses;

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
}

void removeSubstrs(std::string &s, const std::string p) {
  std::string::size_type n = p.length();
  for (std::string::size_type i = s.find(p); i != std::string::npos;
       i = s.find(p))
    s.erase(i, n);
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

  if(!sockets_.cmd || !sockets_.cmd->is_open()){
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
      if (cmd_resp_queue_.front().find(key) != std::string::npos) {
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
          out.ack = true;
        } else {
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
  header_buffers_.water_column.assign(0);

  sockets_.water_column->async_receive(
      boost::asio::buffer(header_buffers_.water_column), 0,
      boost::bind(&NorbitConnection::wcHandler, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void NorbitConnection::wcHandler(const boost::system::error_code &error,
                                  std::size_t bytes_transferred) {

  processHdrMsg(*sockets_.water_column,header_buffers_.water_column);
  receiveWC();
  return;
}

void NorbitConnection::receiveBathy() {
  header_buffers_.bathymetric.assign(0);
  sockets_.bathymetric->async_receive(
      boost::asio::buffer(header_buffers_.bathymetric), 0,
      boost::bind(&NorbitConnection::bathyHandler, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void NorbitConnection::bathyHandler(const boost::system::error_code &error,
                                  std::size_t bytes_transferred) {

  processHdrMsg(*sockets_.bathymetric,header_buffers_.bathymetric);
  receiveBathy();
  return;
}

void NorbitConnection::processHdrMsg(boost::asio::ip::tcp::socket & sock, boost::array<char, sizeof(norbit_interfaces::msg::CommonHeader)> &hdr){
  norbit_types::Message msg;
  if (msg.fromBoostArray(hdr)) {
    const unsigned int dataSize = msg.commonHeader().size - sizeof(norbit_interfaces::msg::CommonHeader);
    std::shared_ptr<char> dataPtr;
    dataPtr.reset(new char[dataSize]);
    size_t bytesRead =read(sock,boost::asio::buffer(dataPtr.get(), dataSize));

    if(msg.setBits(dataPtr)){
      if (msg.commonHeader().type == norbit_types::bathymetric) {
        bathyCallback(msg.getBathy());
      }
      if (msg.commonHeader().type == norbit_types::watercolum){
          wcCallback(msg.getWC());
      }
    }
  }

}


void NorbitConnection::bathyCallback(norbit_types::BathymetricData data) {
  for(auto &cb : bathymetry_callbacks_) {
    cb(data);
  }

}

void NorbitConnection::wcCallback(norbit_types::WaterColumnData data){
  for(auto &cb : watercolumn_callbacks_) {
    cb(data);
  }
}



void NorbitConnection::spin_once() {
  io_service_.run_for(100ms);
}





