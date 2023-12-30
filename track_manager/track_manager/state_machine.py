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
from movebot_interfaces.srv import TrackPoints, ScanPoints, PrintFile

# define all the possible states
class State(Enum):
    WAITING = auto()
    HOME = auto()
    SCAN = auto()
    PRINTING = auto()
    GRASP = auto()
    TRACK = auto()
    PLACE = auto()

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')

        ########PARAMETERS########
        self.scan_region = [[0.0, 0.0, False], [1.0, 0.0, False]] # define the scan region
        self.tolerance = 0.2 # define the tolerance for the scan region, the unit is in m
        self.printer_scan_location = [0.0,0.0]
        ##########################

        # initialize the machine
        self.state = State.WAITING
        self.in_motion = False
        self.prev_state = None
        self.print_name = None
        self.cbgroup = ReentrantCallbackGroup()
        self.cbgroup_timer = MutuallyExclusiveCallbackGroup()
        self.all_region_scanned = False
        self.tag_location = [] # list of tag locations
        self.track_end = [] # list of track end locations
        self.left_pointer = 0 # left pointer for the tag location
        self.right_pointer = 1 # right pointer for the tag location
        self.initial_scan = True
        self.track_end_generated_last = []
        self.end_left = []
        self.start_right = []

        # open points.txt and read the points
        self.current_dir = os.getcwd()
        with open(self.current_dir + "/src/track_manager/track_manager/points.txt", "r") as f:
            for line in f:
                # split the line by space
                line = line.split(" ")

                # convert it to float for the tag location
                # note that this is in the camera frame 
                tag = [float(line[0]), float(line[1]), float(line[2])]
                self.tag_location.append(tag)

                
        with open(self.current_dir + "/src/track_manager/track_manager/track_end.txt", "r") as f:
            for line in f:
                # split the line by space
                # note that this in the track frame
                line = line.split(" ")
                track_end = [float(line[0]), float(line[1]), float(line[2])]
                self.track_end.append(track_end)   

        # define service
        self.home_service = self.create_client(SetBool, 'home_service')
        self.start_robot_service = self.create_service(Empty, 'start_robot_service', self.start_robot_callback)
        self.scan_service = self.create_client(ScanPoints, 'scan_service')
        self.track_service = self.create_client(TrackPoints, 'track_service') 
        self.print_service = self.create_client(PrintFile, 'print_service') 
        self.grasp_service = self.create_client(ScanPoints, 'grasp_service') 
        self.place_service = self.create_client(ScanPoints, 'place_service') 
        self.get_logger().info('starting services')
        self.home_service.wait_for_service()
        self.scan_service.wait_for_service()
        self.track_service.wait_for_service()
        self.grasp_service.wait_for_service()
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
        elif self.state == State.PLACE:
            await self.place()

    def start_robot_callback(self, request, response):
        self.state = State.HOME
        self.get_logger().info("start robot")
        return response
    
    
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
        # print out the scan region
        self.get_logger().info("scan region: " + str(self.scan_region))
        for i in range(0, len(self.scan_region)):
            if self.scan_region[i][2] == True:
               continue
            else:
                point_scan_next = self.scan_region[i]
                # set the original point to true
                self.scan_region[i][2] = True
                break
        # transition to track if all the points have been scanned
        if point_scan_next is None:
            self.all_region_scanned = True
            self.state = State.TRACK
            self.initial_scan = False
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

            # update the files 
            if not exist and self.initial_scan == False:
                tag_pose = [x, y, theta]
                self.tag_location.insert(self.left_pointer,tag_pose)
                self.track_end.insert(self.left_pointer,self.track_end_generated_last)

                 # write the points to the file
                with open(self.current_dir + "/src/track_manager/track_manager/points.txt", "w") as f:
                    f.seek(0)
                    f.truncate()
                    for pose in self.tag_location:
                        f.write(str(pose[0]) + " " + str(pose[1]) + " " + str(pose[2]) + "\n")

                # also change the track end
                with open(self.current_dir + "/src/track_manager/track_manager/track_end.txt", "w") as f:
                    f.seek(0)
                    f.truncate()
                    for track in self.track_end:
                        f.write(str(track[0]) + " " + str(track[1]) + " " + str(track[2]) + "\n")
            
            elif not exist and self.initial_scan == True:
                tag_pose = [x, y, theta]
                self.tag_location.append(tag_pose)
                
                # write the points to the file
                with open(self.current_dir + "/src/track_manager/track_manager/points.txt", "w") as f:
                    for pose in self.tag_location:
                        f.write(str(pose[0]) + " " + str(pose[1]) + " " + str(pose[2]) + "\n")
            
            elif exist:
                with open(self.current_dir + "/src/track_manager/track_manager/points.txt", "w") as f:
                    f.seek(0)
                    f.truncate()
                    for pose in self.tag_location:
                        f.write(str(pose[0]) + " " + str(pose[1]) + " " + str(pose[2]) + "\n")

            else:
                self.get_logger().info("Point already exists")
            if self.future.success:
                self.get_logger().info("Scan service success")
            else:
                self.get_logger().info("Scan service failed")
                self.state = State.WAITING

    async def track(self):
        rqst = TrackPoints.Request()  
        # check if the length of the tag_location is greater than 2
        if len(self.tag_location) < 2:
            self.get_logger().info("Not enough tracks")
            self.state = State.WAITING
            return
        
        # transformation matrix:
        self.end_left = self.global_end(self.tag_location[self.left_pointer], self.track_end[self.left_pointer])
        self.start_right = self.global_start(self.tag_location[self.right_pointer])

        #print out for debugging
        self.get_logger().info("end left: " + str(self.end_left))
        self.get_logger().info("start right: " + str(self.start_right))
        
        tag_pos = [self.end_left[0], self.end_left[1], self.start_right[0], self.start_right[1]]
        tag_angle = [self.end_left[2], self.start_right[2]]

        rqst.position = tag_pos
        rqst.angle = tag_angle

        self.future = await self.track_service.call_async(rqst)
        self.print_name = self.future.file_names
        self.track_end_generated_last = self.future.track_end   

        if self.future.last_piece == True:
            self.right_pointer += 1
            self.left_pointer += 1
        else:
            self.left_pointer += 1

        if self.print_name is not None:
            self.get_logger().info("Track service success")
            self.state = State.PRINTING
        else:
            self.get_logger().info("Track service failed")
            self.state = State.WAITING

    async def printing(self):
        # call the print service
        rqst = PrintFile.Request()
        rqst.file_names = self.print_name
        self.future = await self.print_service.call_async(rqst)
        if self.future.success:
            self.get_logger().info("Print service success")
            self.state = State.GRASP
        
        else:
            self.get_logger().info("Print service failed")
            # reprint the file. NEED TO IMPLEMENT THIS
        
    async def grasp(self):
        rqst = ScanPoints.Request()
        rqst.x = self.printer_scan_location[0]
        rqst.y = self.printer_scan_location[1]
        self.future = await self.grasp_service.call_async(rqst)

        if self.future.success:
            self.get_logger().info("Grasp service success")
            self.state = State.PLACE
            
    async def place(self):
        # perform the transformation
        # call the place service
        rqst = ScanPoints.Request() 

        # compute where to place the object
        self.end_left = self.global_end(self.tag_location[self.left_pointer], self.track_end[self.left_pointer])
        transformation = np.array([[np.cos(self.end_left[2]), -np.sin(self.end_left[2]), self.end_left[0]],
                                   [np.sin(self.end_left[2]), np.cos(self.end_left[2]), self.end_left[1]],
                                   [0.0, 0.0, 1.0]])
        
        place_point = np.array([[5.0], [0.0], [1.0]])
        place_point_transformed = np.matmul(transformation, place_point)
        self.get_logger().info("place point transformed: " + str(place_point_transformed))
        
        
        rqst.x = place_point_transformed[0]
        rqst.y = place_point_transformed[1]
        rqst.theta = self.end_left[2]
        self.future = await self.place_service.call_async(rqst)

        if self.future.success:
            self.get_logger().info("Place service success")
            tag_location = np.array([[5.0], [-5.0], [1.0]])
            tag_location_predicted = np.matmul(transformation, tag_location)
            self.scan_region.append([tag_location_predicted[0], tag_location_predicted[1], False])
            self.state = State.SCAN
    
    ########Helper Functions########
    def global_start(self, tag_location):
        start_tag = [-0.5, 0.5, 1.0]
        matrix_transformation_global2tag = np.array([[np.cos(tag_location[2]), -np.sin(tag_location[2]), tag_location[0]],
                                                     [np.sin(tag_location[2]), np.cos(tag_location[2]), tag_location[1]],
                                                     [0.0, 0.0, 1.0]]) 
        # tag location
        self.get_logger().info("global start tag location: " + str(tag_location))

        start_global = np.matmul(matrix_transformation_global2tag, start_tag)
        # print out the location of the start
        self.get_logger().info("start location in global frame: " + str(start_global))
        start_global = [start_global[0], start_global[1], tag_location[2]] 
        return start_global

    def global_end(self, tag_location, end_location):
        end_tag = [end_location[0], end_location[1], 1.0]
        matrix_transformation_global2tag = np.array([[np.cos(tag_location[2]), -np.sin(tag_location[2]), tag_location[0]],
                                                     [np.sin(tag_location[2]), np.cos(tag_location[2]), tag_location[1]],
                                                     [0.0, 0.0, 1.0]])
        self.get_logger().info("global end tag location: " + str(tag_location))
        end_global = np.matmul(matrix_transformation_global2tag, end_tag)
        # print out the location of the end
        self.get_logger().info("end location in global frame: " + str(end_global))
        end_global = [end_global[0], end_global[1], tag_location[2] + end_location[2]]
        return end_global
    
def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()