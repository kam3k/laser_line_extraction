#include "laser_line_extraction/line_extraction_node.h"
#include <cmath> // sin, cos

namespace line_extraction
{

// Constructor / destructor

LineExtractionNode::LineExtractionNode(ros::NodeHandle& nh, ros::NodeHandle& nh_local):
  nh_(nh),
  nh_local_(nh_local),
  data_cached_(false)
{
  loadParameters();
  line_publisher_ = nh_.advertise<laser_line_extraction::LineSegmentList>("line_segments", 1);
  scan_subscriber_ = nh_.subscribe(scan_topic_, 1, &LineExtractionNode::laserScanCallback, this);
}

LineExtractionNode::~LineExtractionNode()
{
}

// Run

void LineExtractionNode::run()
{
  // Extract the lines
  std::vector<Line> lines;
  line_extraction_.extractLines(lines);

  // Populate message
  laser_line_extraction::LineSegmentList msg;
  populateLineSegListMsg(lines, msg);
  
  // Publish the lines
  line_publisher_.publish(msg);
}

// Load ROS parameters

void LineExtractionNode::loadParameters()
{
  
  ROS_DEBUG("*************************************");
  ROS_DEBUG("PARAMETERS:");

  // Parameters used by this node
  
  std::string frame_id, scan_topic;

  nh_local_.param<std::string>("frame_id", frame_id, "laser");
  frame_id_ = frame_id;
  ROS_DEBUG("frame_id: %s", frame_id_.c_str());

  nh_local_.param<std::string>("scan_topic", scan_topic, "scan");
  scan_topic_ = scan_topic;
  ROS_DEBUG("scan_topic: %s", scan_topic_.c_str());

  // Parameters used by the line extraction algorithm

  double bearing_std_dev, least_sq_angle_thresh, least_sq_radius_thresh, max_line_gap, 
         min_line_length, min_range, min_split_dist, outlier_dist, range_std_dev;
  int min_line_points;
  bool range_std_dev_as_fraction;

  nh_local_.param<double>("bearing_std_dev", bearing_std_dev, 1e-5);
  line_extraction_.setBearingVariance(bearing_std_dev * bearing_std_dev);
  ROS_DEBUG("bearing_std_dev: %0.6f", bearing_std_dev);

  nh_local_.param<double>("least_sq_angle_thresh", least_sq_angle_thresh, 1e-4);
  line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh);
  ROS_DEBUG("least_sq_angle_thresh: %0.6f", least_sq_angle_thresh);
  
  nh_local_.param<double>("least_sq_radius_thresh", least_sq_radius_thresh, 1e-4);
  line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh);
  ROS_DEBUG("least_sq_radius_thresh: %0.6f", least_sq_radius_thresh);

  nh_local_.param<double>("max_line_gap", max_line_gap, 0.4);
  line_extraction_.setMaxLineGap(max_line_gap);
  ROS_DEBUG("max_line_gap: %0.2f", max_line_gap);

  nh_local_.param<double>("min_line_length", min_line_length, 0.5);
  line_extraction_.setMinLineLength(min_line_length);
  ROS_DEBUG("min_line_length: %0.2f", min_line_length);

  nh_local_.param<double>("min_range", min_range, 0.2);
  line_extraction_.setMinRange(min_range);
  ROS_DEBUG("min_range: %0.2f", min_range);

  nh_local_.param<double>("min_split_dist", min_split_dist, 0.05);
  line_extraction_.setMinSplitDist(min_split_dist);
  ROS_DEBUG("min_split_dist: %0.2f", min_split_dist);

  nh_local_.param<double>("outlier_dist", outlier_dist, 0.05);
  line_extraction_.setOutlierDist(outlier_dist);
  ROS_DEBUG("outlier_dist: %0.2f", outlier_dist);

  nh_local_.param<int>("min_line_points", min_line_points, 9);
  line_extraction_.setMinLinePoints(static_cast<unsigned int>(min_line_points));
  ROS_DEBUG("min_line_points: %d", min_line_points);

  nh_local_.param<double>("range_std_dev", range_std_dev, 0.01); // 1% of range by default
  line_extraction_.setRangeStdDev(range_std_dev);
  ROS_DEBUG("range_std_dev: %0.2f", range_std_dev);

  nh_local_.param<bool>("range_std_dev_as_fraction", range_std_dev_as_fraction, true);
  line_extraction_.setRangeStdDevAsFraction(range_std_dev_as_fraction);
  ROS_DEBUG("range_std_dev_as_fraction: %s", range_std_dev_as_fraction ? "true" : "false");

  ROS_DEBUG("*************************************");
}

// Populate messages

void LineExtractionNode::populateLineSegListMsg(const std::vector<Line> &lines,
                                            laser_line_extraction::LineSegmentList &line_list_msg)
{
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    laser_line_extraction::LineSegment line_msg;
    line_msg.angle = cit->getAngle(); 
    line_msg.radius = cit->getRadius(); 
    line_msg.covariance = cit->getCovariance(); 
    line_msg.start = cit->getStart(); 
    line_msg.end = cit->getEnd(); 
    line_list_msg.line_segments.push_back(line_msg);
  }
  line_list_msg.header.frame_id = frame_id_;
  line_list_msg.header.stamp = ros::Time::now();
}

// Cache data on first LaserScan message received

void LineExtractionNode::cacheData(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  std::vector<double> bearings, cos_bearings, sin_bearings;
  std::vector<unsigned int> indices;
  unsigned int i = 0;
  for (double b = scan_msg->angle_min; b <= scan_msg->angle_max; b += scan_msg->angle_increment)
  {
    bearings.push_back(b);
    cos_bearings.push_back(std::cos(b));
    sin_bearings.push_back(std::sin(b));
    indices.push_back(i);
    ++i;
  }

  line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
  ROS_DEBUG("Data has been cached.");
}

// Main LaserScan callback

void LineExtractionNode::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  if (!data_cached_)
  {
    cacheData(scan_msg); 
    data_cached_ = true;
  }

  std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(), scan_msg->ranges.end());
  line_extraction_.setRangeData(scan_ranges_doubles);
}

} // line_extraction namespace

