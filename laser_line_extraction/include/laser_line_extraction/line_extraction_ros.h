#ifndef LINE_EXTRACTION_ROS_H
#define LINE_EXTRACTION_ROS_H

#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "laser_line_extraction_interfaces/msg/line_segment.hpp"
#include "laser_line_extraction_interfaces/msg/line_segment_list.hpp"
#include "laser_line_extraction/line_extraction.h"
#include "laser_line_extraction/line.h"

namespace line_extraction
{

  class LineExtractionROS : public rclcpp::Node
  {

  public:
    // Constructor / destructor
    // LineExtractionROS(ros::NodeHandle &, ros::NodeHandle &);
    LineExtractionROS();
    ~LineExtractionROS();
    // Running
    void run();

  private:
    // ROS
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<laser_line_extraction_interfaces::msg::LineSegmentList>::SharedPtr line_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    // Parameters
    std::string frame_id_;
    std::string scan_topic_;
    bool pub_markers_;
    double frequency_;
    // Line extraction
    rclcpp::TimerBase::SharedPtr timer_;
    LineExtraction line_extraction_;
    bool data_cached_; // true after first scan used to cache data
    // Members
    void loadParameters();
    void populateLineSegListMsg(const std::vector<Line> &, laser_line_extraction_interfaces::msg::LineSegmentList &);
    void populateMarkerMsg(const std::vector<Line> &, visualization_msgs::msg::Marker &);
    void cacheData(const sensor_msgs::msg::LaserScan::SharedPtr &);
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr);
  };

} // namespace line_extraction

#endif
