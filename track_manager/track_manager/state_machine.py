# this node would attempt to connect to the printer 
import rclpy
import time
from rclpy.node import Node
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
from std_srvs.srv import SetBool, Empty
from rclpy.callback_groups import ReentrantCallbackGroup
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
        self.in_motion = False
        self.prev_state = None
        self.cbgroup = ReentrantCallbackGroup()
        self.all_region_scanned = False
        self.tag_location = []
        self.scan_region = np.array([[0.0, 0.0, False], [0.0, 0.0, False]]) # define the scan region
        # define state transitions
       
        # define state functions

        # define service
        self.home_service = self.create_client(SetBool, 'home_service', callback_group=self.cbgroup)
        self.start_robot_service = self.create_service(Empty, 'start_robot_service', self.start_robot_callback)
        self.scan_service = self.create_client(TrackPoints, 'scan_service', callback_group=self.cbgroup) # change the service type
        self.track_service = self.create_client(SetBool, 'track_service', callback_group=self.cbgroup) # change the service type
        
        
        self.home_service.wait_for_service()
        self.scan_service.wait_for_service()
        self.track_service.wait_for_service()
        self.get_logger().info('all services are ready')
        # define publisher

        # define subscriber

        # define action

        # define timer 
        self.timer = self.create_timer(1, self.timer_callback) # update the state every 1 second

        self.get_logger().info("State machine initialized!")

    ########Service Callbacks########
    def timer_callback(self):
        # check if the state has changed
        if self.prev_state != self.state:
            self.prev_state = self.state
            self.get_logger().info("state changed to " + self.state.name)
        
        self.get_logger().info("current state " + self.state.name)

        if self.state == State.WAITING:
            self.waiting()
        elif self.state == State.HOME and self.state != self.prev_state:
            self.home()
        elif self.state == State.SCAN and self.state != self.prev_state:
            self.scan()
        elif self.state == State.TRACK and self.state != self.prev_state:
            self.track()
        elif self.state == State.PRINTING  and self.state != self.prev_state:
            self.printing()
        elif self.state == State.GRASP and self.state != self.prev_state:
            self.grasp()

    def start_robot_callback(self, request, response):
        self.state = State.HOME
        self.get_logger().info("start robot ")
        return response
    
    ########Condition checks########

    
    ########State Functions########
    def waiting(self):
        # do nothing
        pass

    async def home(self):
        # move to home position
        rqst = SetBool.Request()
        rqst.data = True
        self.get_logger().info("calling home service async")
        self.future = self.home_service.call_async(rqst)
        # self.future = await self.home_service.call_async(rqst)

        #rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("home service called")
        if self.future.result().success:
            self.get_logger().info("Home service success")
            self.state = State.SCAN
            return
        else:
            self.get_logger().info("Home service failed")
            self.state = State.WAITING
            return

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
        # call the print service
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