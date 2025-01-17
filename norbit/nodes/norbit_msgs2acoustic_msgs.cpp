#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <norbit/conversions.h>


int main(int argc, char *argv[]) {

  if(argc < 3){
    std::cerr <<  "please specify an input and output bag \nbathymetric2sonar_ranges input.bag output.bag" << std::endl;
    return 1;
  }
  rosbag::Bag in_bag;
  in_bag.open(argv[1]);  // BagMode is Read by default

  rosbag::Bag out_bag;
  out_bag.open(argv[2], rosbag::bagmode::Write);

  marine_acoustic_msgs::SonarDetections::Ptr detections_msg(new marine_acoustic_msgs::SonarDetections);
  marine_acoustic_msgs::SonarRanges::Ptr ranges_msg(new marine_acoustic_msgs::SonarRanges);
  marine_acoustic_msgs::RawSonarImage::Ptr wc_pointer(new marine_acoustic_msgs::RawSonarImage);

  for (rosbag::MessageInstance const m: rosbag::View(in_bag)) {
    norbit_msgs::BathymetricStamped::ConstPtr i = m.instantiate<norbit_msgs::BathymetricStamped>();
    if (i != nullptr) {
      norbit::conversions::bathymetric2SonarDetections(*i, *detections_msg);
      out_bag.write(m.getTopic()+"/mb_detections", m.getTime(), detections_msg);

      norbit::conversions::bathymetric2SonarRanges(*i, *ranges_msg);
      out_bag.write(m.getTopic()+"/sonar_ranges", m.getTime(), ranges_msg);
    }


    norbit_msgs::WaterColumnStamped::ConstPtr wc = m.instantiate<norbit_msgs::WaterColumnStamped>();
    if (wc != nullptr) {
      norbit::conversions::norbitWC2RawSonarImage(*wc, *wc_pointer);
      out_bag.write(m.getTopic()+"/mb_wc", m.getTime(), wc_pointer);
    }
    out_bag.write(m.getTopic(), m.getTime(), m);
  }

  in_bag.close();
  out_bag.close();
  return 0;
}
