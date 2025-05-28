#ifndef WATER_COLUMN_DATA_H
#define WATER_COLUMN_DATA_H

#include "norbit_definitions.h"
#include "norbit_interfaces/msg/water_column_stamped.hpp"

namespace norbit_types {

  class WaterColumnData
  {
  public:
    WaterColumnData();
    size_t dataSize() const;
    void setBits(std::shared_ptr<norbit_interfaces::msg::CommonHeader> comm_hdr, std::shared_ptr<char> bits);
    norbit_interfaces::msg::WaterColumnStamped getRosMsg(std::string frame_id) const;
  protected:
    std::shared_ptr<norbit_interfaces::msg::CommonHeader> comm_hdr_;

    std::shared_ptr<char> bits_;
    norbit_interfaces::msg::WaterColumnHeader * water_column_header_;
    uint8_t * pixel_data_;
    float32 * beam_directions_;
  };
}
#endif // BATHYMETRIC_DATA_H
