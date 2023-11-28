# this node would attempt to generate a track
import rclpy
import time
from rclpy.node import Node
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
import os
# from Track import Track # need to update this

class Track(Node):
    def __init__(self):
        super().__init__('track')

        

        # define 

def main(args=None):
    rclpy.init(args=args)
    node = Track()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()