# this node would attempt to connect to the printer 
import rclpy
import time
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from movebot_interfaces.srv import GetCirclesRqst, GetPoseRqst
import time


class Printer(Node):
     def __init__(self):
        super().__init__('printer')




def main(args=None):
    rclpy.init(args=args)
    node = Printer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()