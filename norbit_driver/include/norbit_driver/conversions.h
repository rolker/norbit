#ifndef TYPE_CONVERER_H
#define TYPE_CONVERER_H

#include "defs.h"
#include "marine_acoustic_msgs/msg/detection_flag.hpp"
#include "marine_acoustic_msgs/msg/raw_sonar_image.hpp"
#include "marine_acoustic_msgs/msg/sonar_ranges.hpp"
#include "marine_acoustic_msgs/msg/sonar_detections.hpp"

#include "norbit_interfaces/msg/water_column_stamped.hpp"
#include "norbit_interfaces/msg/bathymetric_stamped.hpp"

NS_HEAD
namespace conversions {
  /*!
   * \brief Converts norbit_msgs::BathymetricStamped to marine_acoustic_msgs::SonarRanges all parameters passed by reference
   * \param in the norbit_msgs::BathymetricStamped you want to convert
   * \param out the marine_acoustic_msgs::SonarRanges that will be overwritten with the converted Batymetric data
   */
  void bathymetric2SonarRanges(const norbit_interfaces::msg::BathymetricStamped & in, marine_acoustic_msgs::msg::SonarRanges & out);
  void bathymetric2SonarDetections(const norbit_interfaces::msg::BathymetricStamped & in, marine_acoustic_msgs::msg::SonarDetections & out);
  void norbitWC2RawSonarImage(const norbit_interfaces::msg::WaterColumnStamped & in, marine_acoustic_msgs::msg::RawSonarImage & out);
}
NS_FOOT

#endif // TYPE_CONVERER_H
