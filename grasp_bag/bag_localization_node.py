import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import os
import sys
import math
import time
import threading
import matplotlib.pyplot as plt
from grasp_bag.scan_data_sensing_node import ScanDataSensing


class BagLocalization(Node):
    def __init__(self):
        super().__init__('bag_localization_node')
        # Module
        self.sds_node = ScanDataSensing()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.sds_node, ), daemon=True)
        self.thread.start()

    def left_data_estimation(self):
        scan_left_data = self.sds_node.scan_custom_data

    def test_method(self):
        self.sds_node.graph_plot(90)
        result = self.sds_node.scan_range_set(360)
        print(f"result >>> {result}")
        self.thread.join()


def main():
    rclpy.init()
    bl_node = BagLocalization()
    bl_node.test_method()
    bl_node.destroy_node()
    rclpy.shutdown()
