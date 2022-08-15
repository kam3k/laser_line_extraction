#include "laser_line_extraction/line_extraction_ros.h"
#include <cmath>
#include <chrono>
using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"

namespace line_extraction
{

  ///////////////////////////////////////////////////////////////////////////////
  // Constructor / destructor
  ///////////////////////////////////////////////////////////////////////////////

  LineExtractionROS::LineExtractionROS() : Node("line_extraction")
  {
    loadParameters();
    line_publisher_ = this->create_publisher<laser_line_extraction_interfaces::msg::LineSegmentList>("line_segments", 1);
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_, 1, std::bind(&LineExtractionROS::laserScanCallback, this, std::placeholders::_1));
    if (pub_markers_)
    {
      marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("line_markers", 1);
    }
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000.0 / frequency_)), std::bind(&LineExtractionROS::run, this));
  }

  LineExtractionROS::~LineExtractionROS()
  {
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Run
  ///////////////////////////////////////////////////////////////////////////////
  void LineExtractionROS::run()
  {
    // Extract the lines
    std::vector<Line> lines;
    line_extraction_.extractLines(lines);

    // Populate message
    laser_line_extraction_interfaces::msg::LineSegmentList msg;
    populateLineSegListMsg(lines, msg);

    // Publish the lines
    line_publisher_->publish(msg);

    // Also publish markers if parameter publish_markers is set to true
    if (pub_markers_)
    {
      visualization_msgs::msg::Marker marker_msg;
      populateMarkerMsg(lines, marker_msg);
      marker_publisher_->publish(marker_msg);
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Load ROS parameters
  ///////////////////////////////////////////////////////////////////////////////
  void LineExtractionROS::loadParameters()
  {

    RCLCPP_DEBUG(this->get_logger(), "*************************************");
    RCLCPP_DEBUG(this->get_logger(), "PARAMETERS:");

    // Parameters used by this node

    this->declare_parameter<double>("frequency", 25);
    this->get_parameter<double>("frequency", frequency_);

    RCLCPP_DEBUG(this->get_logger(), "frequency: %f", frequency_);

    this->declare_parameter<std::string>("frame_id", "laser");
    this->get_parameter<std::string>("frame_id", frame_id_);

    RCLCPP_DEBUG(this->get_logger(), "frame_id: %s", frame_id_.c_str());

    this->declare_parameter<std::string>("scan_topic", "scan");
    this->get_parameter<std::string>("scan_topic", scan_topic_);

    RCLCPP_DEBUG(this->get_logger(), "scan_topic: %s", scan_topic_.c_str());

    this->declare_parameter<bool>("publish_markers", false);
    this->get_parameter("publish_markers", pub_markers_);

    RCLCPP_DEBUG(this->get_logger(), "publish_markers: %s", pub_markers_ ? "true" : "false");

    // Parameters used by the line extraction algorithm

    double bearing_std_dev, range_std_dev, least_sq_angle_thresh, least_sq_radius_thresh,
        max_line_gap, min_line_length, min_range, max_range, min_split_dist, outlier_dist;
    int min_line_points;

    this->declare_parameter<double>("bearing_std_dev", 1e-3);
    this->get_parameter<double>("bearing_std_dev", bearing_std_dev);
    line_extraction_.setBearingVariance(bearing_std_dev * bearing_std_dev);
    RCLCPP_DEBUG(this->get_logger(), "bearing_std_dev: %f", bearing_std_dev);

    this->declare_parameter<double>("range_std_dev", 0.02);
    this->get_parameter<double>("range_std_dev", range_std_dev);
    line_extraction_.setRangeVariance(range_std_dev * range_std_dev);
    RCLCPP_DEBUG(this->get_logger(), "range_std_dev: %f", range_std_dev);

    this->declare_parameter<double>("least_sq_angle_thresh", 1e-4);
    this->get_parameter("least_sq_angle_thresh", least_sq_angle_thresh);
    line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh);
    RCLCPP_DEBUG(this->get_logger(), "least_sq_angle_thresh: %f", least_sq_angle_thresh);

    this->declare_parameter<double>("least_sq_radius_thresh", 1e-4);
    this->get_parameter("least_sq_radius_thresh", least_sq_radius_thresh);
    line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh);
    RCLCPP_DEBUG(this->get_logger(), "least_sq_radius_thresh: %f", least_sq_radius_thresh);

    this->declare_parameter<double>("max_line_gap", 0.4);
    this->get_parameter("max_line_gap", max_line_gap);
    line_extraction_.setMaxLineGap(max_line_gap);
    RCLCPP_DEBUG(this->get_logger(), "max_line_gap: %f", max_line_gap);

    this->declare_parameter<double>("min_line_length", 0.5);
    this->get_parameter("min_line_length", min_line_length);
    line_extraction_.setMinLineLength(min_line_length);
    RCLCPP_DEBUG(this->get_logger(), "min_line_length: %f", min_line_length);

    this->declare_parameter<double>("min_range", 0.4);
    this->get_parameter("min_range", min_range);
    line_extraction_.setMinRange(min_range);
    RCLCPP_DEBUG(this->get_logger(), "min_range: %f", min_range);

    this->declare_parameter<double>("max_range", 10000.0);
    this->get_parameter("max_range", max_range);
    line_extraction_.setMaxRange(max_range);
    RCLCPP_DEBUG(this->get_logger(), "max_range: %f", max_range);

    this->declare_parameter<double>("min_split_dist", 0.05);
    this->get_parameter("min_split_dist", min_split_dist);
    line_extraction_.setMinSplitDist(min_split_dist);
    RCLCPP_DEBUG(this->get_logger(), "min_split_dist: %f", min_split_dist);

    this->declare_parameter<double>("outlier_dist", 0.05);
    this->get_parameter("outlier_dist", outlier_dist);
    line_extraction_.setOutlierDist(outlier_dist);
    RCLCPP_DEBUG(this->get_logger(), "outlier_dist: %f", outlier_dist);

    this->declare_parameter<int>("min_line_points", 9);
    this->get_parameter("min_line_points", min_line_points);
    line_extraction_.setMinLinePoints(static_cast<unsigned int>(min_line_points));
    RCLCPP_DEBUG(this->get_logger(), "min_line_points: %d", min_line_points);

    RCLCPP_DEBUG(this->get_logger(), "*************************************");
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Populate messages
  ///////////////////////////////////////////////////////////////////////////////
  void LineExtractionROS::populateLineSegListMsg(const std::vector<Line> &lines,
                                                 laser_line_extraction_interfaces::msg::LineSegmentList &line_list_msg)
  {
    for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
    {
      laser_line_extraction_interfaces::msg::LineSegment line_msg;
      line_msg.angle = cit->getAngle();
      line_msg.radius = cit->getRadius();
      line_msg.covariance = cit->getCovariance();
      line_msg.start = cit->getStart();
      line_msg.end = cit->getEnd();
      line_list_msg.line_segments.push_back(line_msg);
    }
    line_list_msg.header.frame_id = frame_id_;
    line_list_msg.header.stamp = this->now();
  }

  void LineExtractionROS::populateMarkerMsg(const std::vector<Line> &lines,
                                            visualization_msgs::msg::Marker &marker_msg)
  {
    marker_msg.ns = "line_extraction";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker_msg.scale.x = 0.1;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;
    for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
    {
      geometry_msgs::msg::Point p_start;
      p_start.x = cit->getStart()[0];
      p_start.y = cit->getStart()[1];
      p_start.z = 0;
      marker_msg.points.push_back(p_start);
      geometry_msgs::msg::Point p_end;
      p_end.x = cit->getEnd()[0];
      p_end.y = cit->getEnd()[1];
      p_end.z = 0;
      marker_msg.points.push_back(p_end);
    }
    marker_msg.header.frame_id = frame_id_;
    marker_msg.header.stamp = this->now();
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Cache data on first LaserScan message received
  ///////////////////////////////////////////////////////////////////////////////
  void LineExtractionROS::cacheData(const sensor_msgs::msg::LaserScan::SharedPtr &scan_msg)
  {
    std::vector<double> bearings, cos_bearings, sin_bearings;
    std::vector<unsigned int> indices;
    const std::size_t num_measurements = std::ceil(
        (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
    for (std::size_t i = 0; i < num_measurements; ++i)
    {
      const double b = scan_msg->angle_min + i * scan_msg->angle_increment;
      bearings.push_back(b);
      cos_bearings.push_back(cos(b));
      sin_bearings.push_back(sin(b));
      indices.push_back(i);
    }

    line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
    RCLCPP_DEBUG(this->get_logger(), "Data has been cached.");
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Main LaserScan callback
  ///////////////////////////////////////////////////////////////////////////////
  void LineExtractionROS::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    if (!data_cached_)
    {
      cacheData(scan_msg);
      data_cached_ = true;
    }

    std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(), scan_msg->ranges.end());
    line_extraction_.setRangeData(scan_ranges_doubles);
  }

} // namespace line_extraction
