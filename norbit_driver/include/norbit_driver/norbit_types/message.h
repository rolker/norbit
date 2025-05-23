#ifndef MESSAGE_H
#define MESSAGE_H

#include <iostream>
#include <cstring>
#include <cstdint>
//#include "header.h"
#include "bathymetric_data.h"
#include "norbit_driver/norbit_types/water_column_data.h"
#include "norbit_interfaces/msg/common_header.hpp"
#include "CRC.h"
namespace norbit_types {
class Message
  {
  public:
    Message(){return;}
    bool fromBoostArray(boost::array<char, sizeof(norbit_interfaces::msg::CommonHeader)> array){
      header_.reset(new norbit_interfaces::msg::CommonHeader);
      memcpy(header_.get(),&array,sizeof(norbit_interfaces::msg::CommonHeader));
      return headerValid();
    }
    norbit_interfaces::msg::CommonHeader commonHeader(){return *header_;}
    bool headerValid(){
      return header_->preable==norbit_interfaces::msg::CommonHeader::NORBIT_PREAMBLE_KEY &&
             header_->version==NORBIT_CURRENT_VERSION;
    }
    bool setBits(std::shared_ptr<char> bits){
      bits_ = bits;
      if(msgValid()){
        switch (header_->type) {
        case bathymetric:
          bathy_.setBits(header_,bits);
          break;
        case watercolum:
          water_column_.setBits(header_,bits);
          break;
        default:
            std::cerr << "unable to read data:  unknown/undefined type"  << std::endl;
          break;
        }
        return  true;
      }else {
        return false;
      }
    }
    bool msgValid(){
      std::uint32_t crc = CRC::Calculate(bits_.get(), header_->size - sizeof(norbit_interfaces::msg::CommonHeader) , CRC::CRC_32());
      return  crc==header_->crc;
    }
    BathymetricData getBathy(){return bathy_;}
    WaterColumnData getWC(){return water_column_;}

  protected:
    std::shared_ptr<char> bits_;
    std::shared_ptr<norbit_interfaces::msg::CommonHeader> header_;
    BathymetricData bathy_;
    WaterColumnData water_column_;

  };
}
#endif // MESSAGE_H
