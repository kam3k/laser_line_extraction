#!/usr/bin/env python

# ROS imports
import rospy
from laser_line_extraction.msg import LineSegment, LineSegmentList
from sensor_msgs.msg import LaserScan

# Package imports
from laser_line_extraction import extract_lines 

# Python imports
import threading

class LLENode(object):

    def __init__(self):
        rospy.init_node("line_extractor")
        self.bearings = []
        self.ranges = []
        self.new_data = False
        self.lock = threading.Lock()
        self.publisher = rospy.Publisher('/line_segments', LineSegmentList)
        rospy.Subscriber('/sick/lms111/scan', LaserScan, self.process_scan, queue_size = 1)

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
            self.new_data = True

    def run(self):
        while not rospy.is_shutdown():
            if self.new_data:
                with self.lock:
                    extract_lines.set_data(self.ranges[:], self.bearings[:])
                line_list = extract_lines.run()
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

