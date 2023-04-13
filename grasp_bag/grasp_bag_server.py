import time
import rclpy
import threading
from rclpy.node import Node
from happymini_msgs.srv import GraspBag, BagLocalization
from happymini_teleop.base_control import BaseControl


class GraspBagServer(Node):
    def __init__(self):
        super().__init__('grasp_bag_node')
        # Service
        self.create_service(GraspBag, 'grasp_bag_server', self.execute)
        self.bl_srv = self.create_client(BagLocalization, 'bag_localization_server')
        while not self.bl_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("bag_localization_server is not here ...")
        self.bl_srv_req = BagLocalization.Request()
        # Module
        self.bc_node = BaseControl()
        self.bc_thread = threading.Thread(target=rclpy.spin, args=(self.bc_node, ), daemon=True)
        self.bc_thread.start()

    def execute(self):#, srv_req, srv_res):
        self
        self.bl_srv_req.left_right = 'all'
        self.bl_srv_req.degree = 180
        self.bl_srv_req.graph = False
        result = self.bl_srv.call_async(self.bl_srv_req)
        print(f"bl_srv_result >>> {result}")


def main():
    rclpy.init()
    gbs = GraspBagServer()
    gbs.execute()
    gbs.destroy_node()
    rclpy.shutdown()
