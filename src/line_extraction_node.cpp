#include "laser_line_extraction/line_extraction_ros.h"
#include <ros/console.h>

int main(int argc, char **argv)
{

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
  {
     ros::console::notifyLoggerLevelsChanged();
  }

  ROS_DEBUG("Starting line_extraction_node.");

  ros::init(argc, argv, "line_extraction_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");
  line_extraction::LineExtractionROS line_extractor(nh, nh_local);

  double frequency;
  nh_local.param<double>("frequency", frequency, 25);
  ROS_DEBUG("Frequency set to %0.1f Hz", frequency);
  ros::Rate rate(frequency);

  while (ros::ok())
  {
    line_extractor.run();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

