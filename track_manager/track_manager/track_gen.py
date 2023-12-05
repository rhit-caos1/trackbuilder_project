# this node would attempt to generate a track
import rclpy
import time
from rclpy.node import Node
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
import os
from Global_connect import gen_train_curves
from movebot_interfaces.srv import TrackPoints

class Track(Node):
    def __init__(self):
        super().__init__('track')
        self.get_logger().info("Track node started.")

        # define 

        self.track_connect = self.create_client(TrackPoints, 'track_points', self.connect_callback)

    async def connect_callback(self, request, response):
        if len(request.position) != 4 or len(request.angle) != 2:
            self.get_logger().info("Invalid request")
            response.success = False
            return response
        
        first_point_pos = np.array([[request.position[0], request.position[1]]])
        first_point_angle = np.array(request.angle[0])
        second_point_pos = np.array([[request.position[2], request.position[3]]])
        second_point_angle = np.array(request.angle[1])
        self.get_logger().info("Start generating track")
        track_points = gen_train_curves(first_point_pos, first_point_angle, second_point_pos, second_point_angle)
        status, filenames = self.generate_scad(track_points)

        if status:
            response.success = True
            response.file_names = filenames
        else:
            response.success = False
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Track()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()