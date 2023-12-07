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

# define all the possible states
class State(Enum):
    WAITING = auto()
    HOME = auto()
    SCAN = auto()
    PRINTING = auto()
    GRASP = auto()
    TRACK = auto()

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        # initialize the machine
        self.state = State.WAITING
        self.all_region_scanned = False
        self.tag_location = []
        self.scan_region = np.array([[0.0, 0.0, False], [0.0, 0.0, False]]) # define the scan region
        # define state transitions
       
        # define state functions

        # define service
        self.home_service = self.create_client(SetBool, 'home_service')
        self.start_robot_service = self.create_service(SetBool, 'start_robot_service', self.start_robot_callback)
        self.scan_service = self.create_client(TrackPoints, 'scan_service') # change the service type
        self.track_service = self.create_client(SetBool, 'track_service') # change the service type
    
        # define publisher

        # define subscriber

        # define action

        # define timer 
        self.timer = self.create_timer(1, self.timer_callback) # update the state every 1 second

        self.get_logger().info("State machine initialized!")
        self.get_logger().info("current state " + self.state.name)
    
    ########Service Callbacks########
    def timer_callback(self):
        self.get_logger().info("current state " + self.state.name)
        if self.state == State.WAITING:
            self.waiting()
        elif self.state == State.HOME:
            self.home()
        elif self.state == State.SCAN:
            self.scan()
        elif self.state == State.TRACK:
            self.track()
        elif self.state == State.PRINTING:
            self.printing()
        elif self.state == State.GRASP:
            self.grasp()

    def start_robot_callback(self, request, response):
        if request.data:
            self.state = State.HOME
            response.success = True
            response.message = "Robot started"
        else:
            response.success = False
            response.message = "Robot failed to start"
        return response
    
    ########Condition checks########

    
    ########State Functions########
    def waiting(self):
        # do nothing
        pass

    def home(self):
        # move to home position
        rqst = SetBool.Request()
        rqst.data = True
        response = self.home_service.call(rqst)
        if response.success:
            self.get_logger().info("Home service success")
            self.state = State.SCAN
        else:
            self.get_logger().info("Home service failed")
            self.state = State.WAITING

    def scan(self):
        # loop through the scan region
        point_scan_next = None
        for point in self.scan_region:
            if point[2]:
               continue
            else:
                point_scan_next = point
                self.scan_region[point] = True #
        
        if point_scan_next is None:
            self.all_region_scanned = True
            self.state = State.TRACK
            return 
        
        else:
            # call the scan service
            rqst = TrackPoints.Request()
            rqst.x = point_scan_next[0]
            rqst.y = point_scan_next[1]
            response = self.scan_service.call(rqst)
            # save this response to a file
            
            # TO DO: do a LMSR to update the points 
            self.tag_location.append([response.x, response.y, response.theta])
            open("points.txt", "a").write(response)
            if response.success:
                self.get_logger().info("Scan service success")
            else:
                self.get_logger().info("Scan service failed")
                self.state = State.WAITING


    def track(self):
        rqst = SetBool.Request()   
        rqst.data = True
        response = self.track_service.call(rqst)
        if response.success:
            self.get_logger().info("Track service success")
            self.state = State.PRINTING
        else:
            self.get_logger().info("Track service failed")
            self.state = State.WAITING


    def printing(self):
        pass

    def grasp(self):
        pass

        # self.future = self.home_service.call_async(rqst)
        # rclpy.spin_until_future_complete(self, self.home_service)
        # if self.future.result() is not None:
        #     self.get_logger().info("Home service success")
        #     self.state = State.WAITING
        # else:
        #     self.get_logger().info("Home service failed")
        # pass

def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()