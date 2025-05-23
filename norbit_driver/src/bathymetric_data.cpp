#include "norbit_driver/norbit_types/bathymetric_data.h"
#include "rclcpp/rclcpp.hpp"

namespace norbit_types {
  BathymetricData::BathymetricData()
  {

  }

  void BathymetricData::setBits(std::shared_ptr<norbit_interfaces::msg::CommonHeader> comm_hdr, std::shared_ptr<char> bits){
    bits_ = bits;
    comm_hdr_ = comm_hdr;
    bathymetric_header_ = reinterpret_cast<norbit_interfaces::msg::BathymetricHeader*>(
          bits_.get() );
    data_ = reinterpret_cast<norbit_interfaces::msg::BathymetricPoint*>(
          &bits_.get()[sizeof(norbit_interfaces::msg::BathymetricHeader)] );
  }
  norbit_interfaces::msg::BathymetricStamped BathymetricData::getRosMsg(std::string frame_id){
    norbit_interfaces::msg::BathymetricStamped outMsg;
    rclcpp::Time stamp(bathymetric_header_->time*1000000000);
    outMsg.header.stamp = stamp;
    outMsg.header.frame_id = frame_id;
    outMsg.bathy.common_header = *comm_hdr_;
    outMsg.bathy.bathymetric_header = *bathymetric_header_;
    outMsg.bathy.detections.assign(data_,data_+bathymetric_header_->n);
    return outMsg;
  }
}
