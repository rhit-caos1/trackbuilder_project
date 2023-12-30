# this node would attempt to connect to the printer 
import rclpy
import time
from rclpy.node import Node
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
# using the octorest library. we do have our own implementation of the octorest library, but it is not working
from octorest import OctoRest
from std_srvs.srv import SetBool, Empty
from movebot_interfaces.srv import PrintFile, PrinterStatus
import os


class Printer(Node):
    def __init__(self):
        super().__init__('printer')
        self.API_KEY = "FD9AFFAB43284EC2B6E838019CF41344"
        self.base_address = "http://10.106.4.208"
        self.client = None
        self.path = None
        self.current_dir = os.getcwd()

        # define service
        self.connect_service = self.create_service(Empty, 'connect', self.connect_callback)
        self.upload_service = self.create_service(PrintFile, 'upload', self.upload_callback)
        self.disconnect_service = self.create_service(Empty, 'disconnect', self.disconnect_callback)
        self.print_service = self.create_service(SetBool, 'print', self.print_callback)
        self.check_print = self.create_service(PrinterStatus, 'check_print', self.check_print_callback)
    
    def connect_callback(self, request, response):
        try:
            self.client = OctoRest(url=self.base_address, apikey=self.API_KEY)
            if self.client is None:
                self.get_logger().info("Failed to connect to printer!")
                raise Exception("Failed to connect to printer!")
            else:
                self.get_logger().info("Connected to printer!")
            
            return response

        except Exception as e:
            self.get_logger().info("Failed to connect to printer!")
            self.get_logger().info(e)
            return response

    def disconnect_callback(self, request, response):
        self.client.disconnect()
        self.client = None
        self.get_logger().info("Disconnected from printer!")
        return response

    def upload_callback(self, request, response):
        file_path = request.file_names
        self.path = file_path
        # for testing 
        file_path = self.current_dir + "/ws_train/src/track_manager/track_manager/gcode/testing1228.gcode"
        self.path = file_path
        self.get_logger().info(file_path)
        # check if file exists

        if not os.path.exists(file_path):
            self.get_logger().info("File does not exist!")
            response.success = False
            return response
    
        try:
            if self.client is None:
                self.connect()
            self.client.upload(file_path)
            response.success = True
            self.get_logger().info("Uploaded file to printer!")
            return response
        except Exception as e:
            self.get_logger().info("Failed to upload file to printer!")
            self.get_logger().info(e)
            response.success = False
            return response
    
    def print_callback(self, request, response):
        if request.data is False:
            # return 
            response.success = False
            return response

        try:
            if self.client is None:
                raise Exception("Printer not connected!")
            self.client.select(self.path, print=True)
            self.get_logger().info("Printing file!")
            response.data = True
            return response
        except:
            self.get_logger().info("Failed to print file!")
            response.data = False
            return response

    def check_print_callback(self, request, response):
        try:
            self.get_logger().info("Checking printer status!")
            res = self.client.job_info()
            # self.get_logger().info(str(res))
            status = str(res['state'])
            self.get_logger().info(status)
            
            if status == "Printing":
                response.printing = True
                time_left = res['progress']['printTimeLeft']
                response.time_left = float(time_left)

            elif status == "Operational":
                response.printing = False
                response.time_left = float(-1)
            
            elif status == "Error" or status == "Offline" or status == "Offline after error":
                response.printing = False
                response.time_left = float(-2)
            
            if response.printing:
                self.get_logger().info("Printer is printing!")
            else:
                self.get_logger().info("Printer is not printing!")

            return response
        except Exception as e:
            self.get_logger().info("Failed to get printer status!")
            self.get_logger().info(e)
            return response
        
def main(args=None):
    rclpy.init(args=args)
    node = Printer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()