# this node would attempt to connect to the printer 
import rclpy
import time
from rclpy.node import Node
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
from std_srvs.srv import SetBool
import os
from enum import Enum, auto
from movebot_interfaces.srv import TrackPoints



class Test(Node):
    def __init__(self):
        super().__init__('Test')

        self.home_service = self.create_service(SetBool, 'home_service', self.home_callback)
        self.scan_service = self.create_service(TrackPoints, 'scan_service', self.scan_callback)
        self.track_service = self.create_service(SetBool, 'track_service', self.track_callback)
    

    def home_callback(self, request, response):
        self.get_logger().info("home callback")
        response.success = True
        response.message = "home success"
        self.get_logger().info("complete")
        return response
    
    def scan_callback(self, request, response):
        self.get_logger().info("scan callback")
        response.x = 0 
        response.y = 0
        response.theta = 0
        return response
    
    def track_callback(self, request, response):
        self.get_logger().info("track callback")
        response.success = True
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = Test()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()