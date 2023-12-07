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

        self.home_service = self.create_client(SetBool, 'home_service')
        self.start_robot_service = self.create_server(SetBool, 'start_robot_service', self.start_robot_callback)
        self.scan_service = self.create_client(TrackPoints, 'scan_service') # change the service type
        self.track_service = self.create_client(SetBool, 'track_service') # change the service type
    
def main(args=None):
    rclpy.init(args=args)
    node = Test()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()