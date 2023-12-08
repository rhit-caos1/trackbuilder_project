# this node would attempt to connect to the printer 
import rclpy
import time
from rclpy.node import Node
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
from std_srvs.srv import SetBool, Empty
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import os
from enum import Enum, auto
from movebot_interfaces.srv import TrackPoints, ScanPoints

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
        self.print_name = None
        self.cbgroup = ReentrantCallbackGroup()
        self.cbgroup_timer = MutuallyExclusiveCallbackGroup()
        self.all_region_scanned = False
        self.tag_location = [] # list of tag locations
        # open points.txt and read the points
        self.current_dir = os.getcwd()
        with open(self.current_dir + "/src/track_manager/track_manager/points.txt", "r") as f:
            for line in f:
                self.tag_location.append(line.split(" "))

        self.scan_region = np.array([[0.0, 0.0, False], [0.0, 0.0, False]]) # define the scan region
        self.tolerance = 0.3 # define the tolerance for the scan region, the unit is in m
        
        # define state transitions
       
        # define state functions

        # define service
        self.home_service = self.create_client(SetBool, 'home_service')
        self.start_robot_service = self.create_service(Empty, 'start_robot_service', self.start_robot_callback)
        self.scan_service = self.create_client(ScanPoints, 'scan_service') # change the service type
        self.track_service = self.create_client(TrackPoints, 'track_service') # change the service type
        self.print_service = self.create_client(SetBool, 'print_service') # change the service type
        
        self.home_service.wait_for_service()
        self.scan_service.wait_for_service()
        self.track_service.wait_for_service()
        self.get_logger().info('all services are ready')
        # define publisher

        # define subscriber

        # define action

        # define timer 
        self.timer = self.create_timer(1, self.timer_callback, callback_group=self.cbgroup_timer) # update the state every 1 second

        self.get_logger().info("State machine initialized!")

    ########Service Callbacks########
    async def timer_callback(self):
        # check if the state has changed
        if self.prev_state != self.state:
            self.prev_state = self.state
            self.get_logger().info("state changed to " + self.state.name)
        
        self.get_logger().info("current state " + self.state.name)

        if self.state == State.WAITING:
            self.waiting()
        elif self.state == State.HOME:
            await self.home()
        elif self.state == State.SCAN:
            await self.scan()
        elif self.state == State.TRACK:
            await self.track()
        elif self.state == State.PRINTING:
            await self.printing()
        elif self.state == State.GRASP:
            await self.grasp()

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
        self.future = await self.home_service.call_async(rqst)
        # self.future = await self.home_service.call_async(rqst)

        #rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("home service called")
        if self.future.success:
            self.get_logger().info("Home service success")
            self.state = State.SCAN
            return
        else:
            self.get_logger().info("Home service failed")
            self.state = State.WAITING
            return

    async def scan(self):
        # loop through the scan region
        point_scan_next = None
        for i in range(0, len(self.scan_region)):
            if self.scan_region[i][2] == True:
               continue
            else:
                point_scan_next = self.scan_region[i]
                # set the original point to true
                self.scan_region[i][2] = True
        # transition to track if all the points have been scanned
        if point_scan_next is None:
            self.all_region_scanned = True
            self.state = State.TRACK
            return 
        
        else:
            # call the scan service
            rqst = ScanPoints.Request()
            rqst.x = point_scan_next[0]
            rqst.y = point_scan_next[1]
            self.future = await self.scan_service.call_async(rqst)
            # save this response to a file
            
            x = self.future.x
            y = self.future.y
            theta = self.future.theta
            
            exist = False
            # TO DO: do a LMSR to update the points 
            for i in range(0, len(self.tag_location)):
                if abs(self.tag_location[i][0] - x) < self.tolerance and abs(self.tag_location[i][1] - y) < self.tolerance:
                    # this means that it is the same point but it has moved
                    self.tag_location[i][0] = x
                    self.tag_location[i][1] = y
                    self.tag_location[i][2] = theta
                    exist = True
                    break
            # check 
            if not exist:
                tag_pose = [x, y, theta]
                self.tag_location.append(tag_pose)
                
                # write the points to the file
                with open(self.current_dir + "/src/track_manager/track_manager/points.txt", "w") as f:
                    for pose in self.tag_location:
                        f.write(str(pose[0]) + " " + str(pose[1]) + " " + str(pose[2]) + "\n")

            if self.future.success:
                self.get_logger().info("Scan service success")
            else:
                self.get_logger().info("Scan service failed")
                self.state = State.WAITING

    async def track(self):
        rqst = TrackPoints.Request()  
        # TO DO: need to keep track of what tags have already been generated and printed
        self.future = await self.track_service.call_async(rqst)
        self.print_name = self.future.file_names

        if self.future.last_piece == True:
            # that means we have finished printing all the pieces for this tag
            # TODO: manage the tag
            pass

        if self.print_name is not None:
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