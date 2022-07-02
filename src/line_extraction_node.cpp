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
  ros::spin();
  return 0;
}

