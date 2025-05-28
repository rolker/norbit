#ifndef BATHYMETRIC_DATA_H
#define BATHYMETRIC_DATA_H

#include "norbit_definitions.h"
#include "norbit_interfaces/msg/bathymetric_stamped.hpp"

namespace norbit_types {

  class BathymetricData
  {
  public:
    BathymetricData();
    void setBits(std::shared_ptr<norbit_interfaces::msg::CommonHeader> comm_hdr, std::shared_ptr<char> bits);
    norbit_interfaces::msg::BathymetricHeader & bathymetricHeader(){return *bathymetric_header_;}
    const norbit_interfaces::msg::BathymetricHeader & bathymetricHeader() const {return *bathymetric_header_;}
    norbit_interfaces::msg::BathymetricPoint & data(size_t i){return data_[i];}
    const norbit_interfaces::msg::BathymetricPoint & data(size_t i) const {return data_[i];}
    norbit_interfaces::msg::BathymetricStamped getRosMsg(std::string frame_id) const;
  protected:
    std::shared_ptr<norbit_interfaces::msg::CommonHeader> comm_hdr_;

    std::shared_ptr<char> bits_;
    norbit_interfaces::msg::BathymetricHeader * bathymetric_header_;
    norbit_interfaces::msg::BathymetricPoint * data_;

  };
}
#endif // BATHYMETRIC_DATA_H
