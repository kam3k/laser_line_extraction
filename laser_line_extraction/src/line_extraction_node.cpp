#include "laser_line_extraction/line_extraction_ros.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<line_extraction::LineExtractionROS>());
  rclcpp::shutdown();
  return 0;

  // double frequency;
  // nh_local.param<double>("frequency", frequency, 25);
  // ROS_DEBUG("Frequency set to %0.1f Hz", frequency);
  // ros::Rate rate(frequency);

  // while (ros::ok())
  // {
  //   line_extractor.run();
  //   ros::spinOnce();
  //   rate.sleep();
  // }
  // return 0;
}
