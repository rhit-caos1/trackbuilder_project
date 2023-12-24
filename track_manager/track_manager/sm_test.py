# this node would attempt to connect to the printer 
import rclpy
import time
from rclpy.node import Node
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
from std_srvs.srv import SetBool, Empty
import os
from enum import Enum, auto
from movebot_interfaces.srv import TrackPoints, ScanPoints
import random



class Test(Node):
    def __init__(self):
        super().__init__('Test')
        self.scan_counter = 0
        self.home_service = self.create_service(SetBool, 'home_service', self.home_callback)
        self.scan_service = self.create_service(ScanPoints, 'scan_service', self.scan_callback)
        self.track_service = self.create_service(TrackPoints, 'track_service', self.track_callback)
        self.print_service = self.create_service(SetBool, 'print_service', self.print_callback)
        self.grasp_service = self.create_service(ScanPoints, 'grasp_service', self.grasp_callback)
        self.place_service = self.create_service(ScanPoints, 'place_service', self.place_callback)

    def home_callback(self, request, response):
        self.get_logger().info("home callback")
        self.get_logger().info("complete")
        response.success = True
        return response
    
    def scan_callback(self, request, response):
        self.get_logger().info("scan callback")

        if self.scan_counter == 0:
            response.x = -5.0
            response.y = 5.0  # random int between 0 and 10
            response.theta = 0.0
            response.success = True
        elif self.scan_counter == 1:
            response.x = 5.0
            response.y = 5.0
            response.theta = 0.0
            response.success = True

        self.scan_counter += 1
        return response
    
    def track_callback(self, request, response):
        self.get_logger().info("track callback")
        response.last_piece = False

        return response
    
    def print_callback(self, request, response):
        self.get_logger().info("print callback")
        response.success = True
        return response
    
    def grasp_callback(self, request, response):
        self.get_logger().info("grasp callback")
        response.x = 0.0
        response.y = 0.0
        response.theta = 0.0
        response.success = True
        return response
    
    def place_callback(self, request, response):
        self.get_logger().info("place callback")
        response.x = 0.0
        response.y = 0.0
        response.theta = 0.0
        response.success = True
        return response
    

def main(args=None):
    rclpy.init(args=args)
    node = Test()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()