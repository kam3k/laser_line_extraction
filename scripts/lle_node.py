#!/usr/bin/env python

# ROS imports
import rospy
from laser_line_extraction.msg import LineSegment, LineSegmentList
from sensor_msgs.msg import LaserScan

# Package imports
from laser_line_extraction.line_extractor import LineExtractor

# Python imports
import threading

class LLENode(object):

    def __init__(self):
        rospy.init_node("line_extractor")
        # Get parameters
        scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.range_fraction = rospy.get_param('~range_uncertainty_fraction', True)
        self.range_std_dev = rospy.get_param('~range_std_dev', 0.01)
        self.bearing_std_dev = rospy.get_param('~bearing_std_dev', 0.0001)
        min_range = rospy.get_param('~min_range', 0.2)
        outlier_dist = rospy.get_param('~outlier_dist', 0.05)
        min_split_dist = rospy.get_param('~min_split_dist', 0.05)
        min_line_length = rospy.get_param('~min_line_length', 0.5)
        min_line_points = rospy.get_param('~min_line_points', 9)
        max_line_gap = rospy.get_param('~max_line_gap', 0.4)
        least_sq_angle_thresh = rospy.get_param('~least_sq_angle_thresh', 1e-4)
        least_sq_radius_thresh = rospy.get_param('~least_sq_radius_thresh', 1e-4)
        # Initializations
        self.line_extractor = LineExtractor(min_range, outlier_dist,
                min_split_dist, min_line_length, min_line_points,
                max_line_gap, least_sq_angle_thresh, least_sq_radius_thresh)
        self.bearings = []
        self.ranges = []
        self.bearing_vars = []
        self.range_vars = []
        self.new_data = False
        self.lock = threading.Lock()
        # Publisher and subscriber
        self.publisher = rospy.Publisher('/line_segments', LineSegmentList)
        rospy.Subscriber(scan_topic, LaserScan, self.process_scan, queue_size=1)

    def populate_message(self, line):
        line_seg_msg = LineSegment()
        line_seg_msg.angle = line.a
        line_seg_msg.radius = line.r
        line_seg_msg.covariance = [line.cov_aa, line.cov_ar, line.cov_ar, line.cov_rr]
        line_seg_msg.start = line.start
        line_seg_msg.end = line.end
        return line_seg_msg

    def process_scan(self, msg):
        with self.lock:
            self.bearings = [msg.angle_min + i * msg.angle_increment 
                        for i in range(len(msg.ranges))]
            self.ranges = msg.ranges
            if self.range_fraction:
                self.range_vars = [(self.range_std_dev * r)**2 for r in self.ranges]
            else:
                self.range_vars = [self.range_std_dev**2 for r in self.ranges]
            self.bearing_vars = [self.bearing_std_dev**2 for b in self.bearings]
            self.new_data = True

    def run(self):
        while not rospy.is_shutdown():
            if self.new_data:
                with self.lock:
                    self.line_extractor.set_data(self.ranges[:], self.bearings[:],
                            self.range_vars[:], self.bearing_vars[:])
                line_list = self.line_extractor.run()
                if line_list:
                    line_segment_msgs = []
                    for line in line_list:
                        line_segment_msgs.append(self.populate_message(line))
                    line_segment_list = LineSegmentList()
                    line_segment_list.line_segments = line_segment_msgs
                    line_segment_list.header.stamp = rospy.Time.now()
                    line_segment_list.header.frame_id = '/laser'
                    self.publisher.publish(line_segment_list)
                self.new_data = False

if __name__ == "__main__":
    node = LLENode()
    node.run()

