# this node would attempt to generate a track
import rclpy
import time
from rclpy.node import Node
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
import os
from Global_connect import gen_train_tracks
from subprocess import run
from movebot_interfaces.srv import TrackPoints

class Track(Node):
    def __init__(self):
        super().__init__('track_gen')
        self.get_logger().info("Track node started.")

        # define 

        self.track_connect = self.create_service(TrackPoints, 'track_service', self.connect_callback)

    def connect_callback(self, request, response):
        if len(request.position) != 4 or len(request.angle) != 2:
            self.get_logger().info("Invalid request")
            response.success = False
            return response
        
        first_point_pos = np.array([[request.position[0], request.position[1]]])
        first_point_angle = np.array([request.angle[0]])
        second_point_pos = np.array([[request.position[2], request.position[3]]])
        second_point_angle = np.array([request.angle[1]])
        self.get_logger().info("Start generating track")
        self.get_logger().info(f"Start_pos: {first_point_pos} ")
        self.get_logger().info(f"Start_ang: {first_point_angle} ")
        self.get_logger().info(f"End_pos: {second_point_pos} ")
        self.get_logger().info(f"End_ans: {second_point_angle} ")
        status, filenames,num = gen_train_tracks(first_point_pos, first_point_angle, second_point_pos, second_point_angle, to_stl = True)
        # status, filenames = self.generate_scad(track_points)
        self.get_logger().info(f"File: {filenames} ")
        if status:
            response.success = True
            if num == 1:
                response.last_piece = True
            elif num >1:
                response.last_piece = False
            else:   
                response.success = False
                response.last_piece = False
                self.get_logger().info("wrong num of pieces")
            response.file_names = filenames[0][0]
            stl_name  = f'/home/scg1224/Final_Project/arm_ws/{filenames[0][0]}.stl'
            file_name = os.path.splitext(response.file_names)
            gcode_name = f'/home/scg1224/Final_Project/arm_ws/{filenames[0][0]}.gcode'
            response.file_names = gcode_name
            self.get_logger().info(f"stl File: {stl_name} ")
            self.get_logger().info(f"gcode File: {gcode_name} ")
            result = run(["node",
                 "src/kiri-run/cli",
                 f"--model={stl_name}",
                 f"--output={gcode_name}",
                 "--device=src/cli/kiri-fdm-cr30.json",
                 "--process=src/cli/kiri-fdm-process-cr30.json"],
                 cwd="/home/scg1224/Belt_printer/grid-apps", capture_output=True, text=True)
            self.get_logger().info(f"Finished running ")
            if result.returncode == 0:
                print("Command executed successfully")
                print("Output:", result.stdout)
            else:
                print("Error executing command")
                print("Error message:", result.stderr)
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