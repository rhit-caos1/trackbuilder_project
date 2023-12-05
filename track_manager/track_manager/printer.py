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
from std_srvs.srv import SetBool
from movebot_interfaces.srv import PrintFile
import os


class Printer(Node):
    def __init__(self):
        super().__init__('printer')
        self.API_KEY = "FD9AFFAB43284EC2B6E838019CF41344"
        self.base_address = "http://10.106.4.208"
        self.client = None
        self.path = None

        # define service
        self.connect_service = self.create_service(SetBool, 'connect', self.connect_callback)
        self.upload_service = self.create_service(PrintFile, 'upload', self.upload_callback)
        self.disconnect_service = self.create_service(SetBool, 'disconnect', self.disconnect_callback)
        self.print_service = self.create_service(PrintFile, 'print', self.print_callback)
    
    def connect_callback(self, request, response):
        try:
            self.client = OctoRest(url=self.base_address, apikey=self.API_KEY)
            response.success = True
            self.get_logger().info("Connected to printer!")
            return response
        except Exception as e:
            response.success = False
            self.get_logger().info("Failed to connect to printer!")
            self.get_logger().info(e)
            return response

    def disconnect_callback(self, request, response):
        response.success = True
        self.client.disconnect()
        self.client = None
        self.get_logger().info("Disconnected from printer!")
        return response

    def upload_callback(self, request, response):
        file_path = request.data
        self.path = file_path
        try:
            if self.client is None:
                self.connect()
            self.client.upload(file_path)
            response.data = True
            self.get_logger().info("Uploaded file to printer!")
            return response
        except Exception as e:
            self.get_logger().info("Failed to upload file to printer!")
            self.get_logger().info(e)
            response.data = False
            return response
    
    def print_callback(self, request, response):
        if request.data is None:
            file_name = self.path
        else:
            file_name = request.data

        try:
            if self.client is None:
                self.connect()
            self.client.select(file_name, print=True)
            self.get_logger().info("Printing file!")
            response.data = True
            return response
        except:
            self.get_logger().info("Failed to print file!")
            response.data = False
            return response

    def get_status(self):
        try:
            status = self.client.printer()['state']['flags']['printing']
            if status:
                self.get_logger().info("Printer is printing!")
            else:
                self.get_logger().info("Printer is not printing!")
        except:
            self.get_logger().info("Failed to get printer status!")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = Printer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()