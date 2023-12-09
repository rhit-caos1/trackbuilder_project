import rclpy
import time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Empty
from movebot_interfaces.srv import AddBox, GetPlanRqst, MoveToTrackRqst,GetCirclesRqst,GetPoseRqst
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from franka_msgs.action import Grasp
from franka_msgs.action import Homing
from math import pi
import math
import numpy as np
from types import SimpleNamespace

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import String

def least_sqare_point(l,ref):
    l = np.array(l)
    size = len(l)
    dis = []
    x = ref[0]
    y = ref[1]
    for i in range(size):
        lsd = np.sqrt((l[i][0]-x)**2+(l[i][1]-y)**2)
        dis.append(lsd)
    return np.argmin(dis)

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
      
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z


class PandaControl(Node):

    def __init__(self):
        super().__init__('panda_control')
        self.cbgroup = ReentrantCallbackGroup()
        # Subscribe to object positions from computer vision
        self.track_tag_sub = self.create_subscription(
            Pose, "track_tag_xzy", self.get_track_pose_callback, 10
        )
        # Initialize robot trajectory clients
        self.plan_client = self.create_client(
            GetPlanRqst, "call_plan", callback_group=self.cbgroup
        )
        self.cart_client = self.create_client(
            GetPlanRqst, "call_cart", callback_group=self.cbgroup
        )
        self.execute_client = self.create_client(
            Empty, "call_execute", callback_group=self.cbgroup
        )

        # clients for detection
        self.get_cricles_client = self.create_client(
            GetCirclesRqst, "get_circle", callback_group=self.cbgroup
        )
        self.get_pose_client = self.create_client(
            GetPoseRqst, "get_pose", callback_group=self.cbgroup
        )



        # self.execute_final_path_client = self.create_service(
        #     Empty, "make_hot_chocolate", self.make_chocolate_callback
        # )

        self.request = GetPlanRqst.Request()

        # Initialize gripper action clients
        self._gripper_action_client = ActionClient(
            self, GripperCommand, "/panda_gripper/gripper_action"
        )
        self._grasp_client = ActionClient(self, Grasp, "/panda_gripper/grasp")
        self._homing_client = ActionClient(self, Homing, "/panda_gripper/homing")

        self.define_waypoints()

        self.execute_home_pose_service = self.create_service(
            Empty, "home_robot", self.home_robot
        )

        self.execute_move_pose_service = self.create_service(
            Empty, "move_robot_to_pose", self.move_robot
        )

        self.close_gripper_service = self.create_service(
            Empty, "close_gripper", self.close_gripper_srv
        )

        self.open_gripper_sevice = self.create_service(
            Empty, "open_gripper", self.open_gripper_srv
        )

        self.move_to_track_service = self.create_service(
            MoveToTrackRqst, "move_to_track", self.move_to_track_srv
        )

        self.move_track_cart = self.create_service(
            MoveToTrackRqst, "move_track_cart", self.move_track_cart_srv
        )

        self.grasp_track_sevice = self.create_service(
            Empty, "grasp_track", self.grasp_track_srv
        )

        self.rotate_90_sevice = self.create_service(
            Empty, "rotate_90", self.rotate_90_srv
        )

        self.gripper_force_sevice = self.create_service(
            MoveToTrackRqst, "gripper_force", self.gripper_force_srv
        )


        self.timer = self.create_timer(2, self.timer_callback)
        self.i = 0

        # Create a listener to recieve the TF's from each tag to the camera
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tag_x = 0
        self.tag_y = 0
        self.tag_az = 0

        self.tags_list = []

    def timer_callback(self):
        # try:
        #     tag_2_base = self.tf_buffer.lookup_transform(
        #         'panda_link0',
        #         'tag_0',
        #         rclpy.time.Time())
        #     self.tag_x = tag_2_base.transform.translation.x
        #     self.tag_y = tag_2_base.transform.translation.y
        #     tag_ax,tag_ay,self.tag_az = euler_from_quaternion(
        #         tag_2_base.transform.rotation.x,
        #         tag_2_base.transform.rotation.y,
        #         tag_2_base.transform.rotation.z,
        #         tag_2_base.transform.rotation.w,
        #     )

        # except:
        #     pass
        self.get_logger().info('Publishing: "%s"' % self.i)
        self.i+=1
    
    def get_track_pose_callback(self, pose_msg):
        # Get track pose (track will get from printer / next track)
        self.track_pose = pose_msg

    def get_tag_pose(self):
        try:
            tag_2_base = self.tf_buffer.lookup_transform(
                'panda_link0',
                'tag_0',
                rclpy.time.Time())
            self.tag_x = tag_2_base.transform.translation.x
            self.tag_y = tag_2_base.transform.translation.y
            tag_ax,tag_ay,self.tag_az = euler_from_quaternion(
                tag_2_base.transform.rotation.x,
                tag_2_base.transform.rotation.y,
                tag_2_base.transform.rotation.z,
                tag_2_base.transform.rotation.w,
            )

        except:
            self.get_logger().info("unable to find tf")

    def grasp(self, width, speed=0.01, force=30.0, epsilon=(0.005, 0.005)):
        """
        Grasps an object. It can fail if the width is not accurate

        :param width: width of the object you are grasping
        :param speed: speed the gripper will close
        :param force: force the gripper will grasp the object
        :param epsilon: inner and outer tolerance
        """
        self.get_logger().info("Grasping...")
        goal_msg = Grasp.Goal()
        goal_msg.width = width
        goal_msg.speed = speed
        goal_msg.force = force
        goal_msg.epsilon.inner = epsilon[0]
        goal_msg.epsilon.outer = epsilon[1]
        # # self._grasp_client.wait_for_server()
        # await self._grasp_client.send_goal_async(goal_msg)
        self._grasp_client.wait_for_server()
        self.future_grasp_res= self._grasp_client.send_goal_async(goal_msg)
        self.get_logger().info("Done Grasping")
        # return self.future_grasp_res

    def open_gripper(self):
        """
        Opens the gripper, position=0.04 is open for some reason

        :return: A future object from the ActionClient.send_goal_async()
        function
        """
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.04
        goal_msg.command.max_effort = 1.0
        self._gripper_action_client.wait_for_server()
        self.future_open_res= self._gripper_action_client.send_goal_async(goal_msg)
        self.get_logger().info("Gripper Open")
        return self.future_open_res

    def home_gripper(self):
        """
        Homes the gripper by first closing the gripper, then opening all the
        way.

        :return: A future object from the ActionClient.send_goal_async()
        function
        """
        goal_msg = Homing.Goal()
        self._homing_client.wait_for_server()
        return self._homing_client.send_goal_async(goal_msg)

    async def plan(self,waypoint,execute_now,is_cart=False,is_vect = False):
        """
        Create a trajectory plan for the end-effector to reach thespecified waypoint

        :return: If in x,y,z,r,p,y configuration, a cartesian trajectory plan. If otherwise,
        a rotation motion plan.
        """
        assert(len(waypoint) == 2),'Invalid waypoint recieved'
        self.request.goal_pos.position = waypoint[0]
        self.request.goal_pos.orientation = waypoint[1]

        if len(waypoint[0])>3:
            self.request.is_xyzrpy = False  # SENDING HOME JOINT STATES
        else:
            self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.request.is_cart = is_cart
        self.request.is_vect = is_vect

        self.get_logger().info(f"READY TO PLAN {self.request.is_xyzrpy}")

        await self.plan_client.call_async(self.request)
        # rclpy.spin_until_future_complete(self, future)

        if execute_now and not self.request.goal_pos.orientation :
            await self.send_execute_request()
        elif execute_now and not self.request.goal_pos.position:
            self.get_logger().info(f" FINISHED EXECUTING- CHANGING STATES")
            await self.send_execute_request()
        else:
            self.get_logger().info(f" FINISHED EXECUTING- position and rotation")
            await self.send_execute_request()

    async def send_execute_request(self):
        """
        Execute the trajectory plan - used in each step of the entire
        trajectory sequence.

        :return: Empty
        """
        await self.execute_client.call_async(Empty.Request())
        # rclpy.spin_until_future_complete(self, future)

        self.get_logger().info(f" FINISHED EXECUTING- CHANGING STATES")

        # return future.result()

    def define_waypoints(self):
        """
        Creates a dictionary of waypoints based on the current TF data
        for the different objects. Should be called in timer loop to
        get up to date data.

        Each waypoint must be a nested list of length 2. The first element
        is a list of length 3 corresponding to the waypoint (x,y,z) and the
        second element is another list of length 2 corresponding to the
        (roll, pitch, yaw) of the EE at the waypoint.
        """
        waypoints_dict = {

            "send_home": [

                    [-0.002900171686977362,-0.7843635402847285,-0.0005615004410780063,-2.3563120754991074,0.0032930072986947943,1.5720097393989312,0.7962947704933621, 0.040476, 0.040476],
                    []
                ],
            "new_home": [

                    [pi/2,-0.783508,-0.000244,-2.356829,-0.003843,1.572944,-0.784056, 0.034865, 0.034865],
                    []
                ],
            "new_home2": [

                    [0.00301,-0.783508,-0.000244,-2.356829,-0.003843,1.572944,-0.784056, 0.034865, 0.034865],
                    []
                ],
            "move_test": [[0.3, 0.3, 0.3], []],
            "rotate_home": [[], [pi, 0.0, 0.0]],
            "rotate_90": [
                    [],
                    [pi,0.0,pi/2]
                ],
            "straighten":[
                [],
                [pi,0.0,0.0]
                ],
        }
        self.waypoints = SimpleNamespace(**waypoints_dict)
        
    async def home_robot(self,request,response):
        self.get_logger().info(f" FINISHED EXECUTING home_robot")
        # await self.plan(self.waypoints.new_home, execute_now=True)
        await self.plan(self.waypoints.send_home, execute_now=True)
        # self.get_logger().info(f" FINISHED EXECUTING home_robot")
        # self.plan(self.waypoints.rotate_90, execute_now=True)
        # self.get_logger().info(f" FINISHED EXECUTING home_robot")
        return response
    
    async def move_robot(self,request,response):
        z_0 = 0.48688
        z_1 = 0.15

        correction_num = 0.0002
        cam_off_x = -0.048
        cam_off_y = 0.02

        center_x = 0.30689
        center_y = 0.0
        await self.plan(self.waypoints.send_home, execute_now=True)
        # self.get_tag_pose()
        # # await self.plan([[self.tag_x,self.tag_y,0.3],[]], execute_now=True)
        # # self.get_logger().info(f" FINISHED EXECUTING 1")
        # # time.sleep(3)
        # # await self.plan([[],[pi, 0.0, self.tag_az]],execute_now=True)
        # # time.sleep(3)
        # await self.plan([[self.tag_x,self.tag_y,z_1],[]],execute_now=True)
        # self.get_logger().info(f" FINISHED EXECUTING 2")
        # # await self.plan(self.waypoints.new_home2, execute_now=True)
        # self.get_logger().info(f" FINISHED EXECUTING move_robot_to_pose")
        self.get_logger().info(f" FINISHED EXECUTING home")
        circles_response = await self.get_cricles_client.call_async(GetCirclesRqst.Request())
        self.get_logger().info(f" print response x: {circles_response.x}")
        self.get_logger().info(f" print response y: {circles_response.y}")
        circle_num = len(circles_response.x)
        ##############!!!!!!!!!!!!!!!###################
        # # fix the flipped axis!!!!!!!!!!!!!!!!!
        # # Change it back if camera is fixed!!!!!!!!
        # y_list = circles_response.x
        # circles_response.x = circles_response.y
        # circles_response.y = y_list
        for i in range(circle_num):
            centered = False
            #get x and y in pixel frame
            x_pix = float(circles_response.x[i])
            y_pix = float(circles_response.y[i])
            x = center_x+float(circles_response.x[i])*correction_num*1.5
            y = center_y+float(circles_response.y[i])*correction_num*1.5
            self.get_logger().info(f" expected tag x pix: {x_pix}")
            self.get_logger().info(f" expected tag y pix: {y_pix}")
            self.get_logger().info(f" expected tag x: {x}")
            self.get_logger().info(f" expected tag y: {y}")
            while not centered:
                time.sleep(2)
                # move to next point
                await self.plan([[x,y,z_0],[]],execute_now=True,is_cart=True)
                # save the rotation
                await self.plan([[],[pi,0,0]],execute_now=True)
                time.sleep(2)
                new_circles_response = await self.get_cricles_client.call_async(GetCirclesRqst.Request())
                # ##############!!!!!!!!!!!!!!!###################
                # # fix the flipped axis!!!!!!!!!!!!!!!!!
                # # Change it back if camera is fixed!!!!!!!!
                # y_list = new_circles_response.x
                # new_circles_response.x = new_circles_response.y
                # new_circles_response.y = y_list
                #####################################
                new_circles_pose = np.column_stack((new_circles_response.x,new_circles_response.y))
                self.get_logger().info(f" poses: {new_circles_pose}")
                min_index = least_sqare_point(new_circles_pose,[0,0])
                new_x = float(new_circles_response.x[min_index])
                new_y = float(new_circles_response.y[min_index])
                self.get_logger().info(f" new x: {new_x}")
                self.get_logger().info(f" new y: {new_y}")
                if np.abs(new_x)<30 and np.abs(new_y)<30:
                    centered = True
                else:
                    x = x+new_x*correction_num
                    y = y+new_y*correction_num
                    self.get_logger().info(f" expected tag x: {x}")
                    self.get_logger().info(f" expected tag y: {y}")
            await self.plan([[x,y,z_1],[]],execute_now=True,is_cart=True)
            pose_response = await self.get_pose_client.call_async(GetPoseRqst.Request())
            if pose_response.detected == True:
                self.get_tag_pose()
                self.tags_list.append([self.tag_x,self.tag_y,self.tag_az])
                self.get_logger().info(f" tag detected! x = {self.tag_x}")
                self.get_logger().info(f" tag detected! y = {self.tag_y}")
                self.get_logger().info(f" tag detected! az = {self.tag_az}")
            

        return response
    
    async def grasp_track_srv(self,request,response):
        time.sleep(1)
        self.grasp(width=0.1,force=00.0)
        pose_response = await self.get_pose_client.call_async(GetPoseRqst.Request())
        if pose_response.detected == True:
            self.get_tag_pose()
            self.tags_list.append([self.tag_x,self.tag_y,self.tag_az])
            self.get_logger().info(f" tag detected! x = {self.tag_x}")
            self.get_logger().info(f" tag detected! y = {self.tag_y}")
            self.get_logger().info(f" tag detected! az = {self.tag_az}")

        Transform_matrix = np.array([[1,0,0.03],
                                     [0,1,-0.06],
                                     [0,0,1]])
        Tag_matrix = np.array([[np.cos(self.tag_az),-np.sin(self.tag_az),self.tag_x],
                                [np.sin(self.tag_az),np.cos(self.tag_az),self.tag_y],
                                [0,0,1]])
        end_pose = np.dot(Tag_matrix,Transform_matrix)
        head_x = end_pose[0][2]
        head_y = end_pose[1][2]
        self.get_logger().info(f"track end pose! {end_pose}")
        self.get_logger().info(f"track head! x = {head_x}, y = {head_y}")

        # await self.plan([[self.tag_x+0.03*np.sin(self.tag_az),self.tag_y-0.06*np.cos(self.tag_az),0.15],[]], execute_now=True,is_cart=True)
        await self.plan([[head_x,head_y,0.15],[]], execute_now=True,is_cart=True)
        await self.plan([[],[pi,0.0,self.tag_az]], execute_now=True)
        await self.plan([[head_x,head_y,0.02],[]], execute_now=True,is_cart=True)
        time.sleep(2)
        self.grasp(width=0.04,force=90.0)

        return response

    
    def close_gripper_srv(self,request,response):
        self.grasp(width=0.04,force=90.0)
        return response
    
    def open_gripper_srv(self,request,response):
        # self.open_gripper()
        # self.home_gripper()
        # can be controlled by distance and 0 force
        self.grasp(width=0.08,force=00.0)
        return response

    async def move_to_track_srv(self,request,response):
        x = float(request.x)
        y = float(request.y)
        # az = float(request.az)
        z_0 = 0.15

        az = float(request.yaw)
        # await self.plan([[],[pi,0,pi/2]], execute_now=True)
        await self.plan([[x,y,z_0],[pi,0,0]], execute_now=True)
        # await self.plan([[x,y,z_0],[pi,0,0]], execute_now=True)
        self.get_logger().info(f" FINISHED EXECUTING 1")
        # time.sleep(3)
        # await self.plan([[],[pi, 0.0, yaw]],execute_now=True)
        # time.sleep(3)
        # await self.plan([[x,y,z_0],[]],execute_now=True)
        # self.get_logger().info(f" FINISHED EXECUTING 2")
        # await self.plan([[x,y,z_1],[]],execute_now=True)
        # self.grasp(width=0.04,force=90.0)
        time.sleep(3)
        # await self.plan(self.waypoints.new_home2, execute_now=True)
        return response
    
    async def move_track_cart_srv(self,request,response):
        x = float(request.x)
        y = float(request.y)
        # az = float(request.az)
        z_0 = 0.15

        az = float(request.yaw)
        # await self.plan([[],[pi,0,pi/2]], execute_now=True)
        await self.plan([[x,y,az],[]], execute_now=True,is_cart=True)
        return response

    async def rotate_90_srv(self,request,response):
        # rotate end effector 90 deg
        await self.plan([[],[pi,0,-pi/2]], execute_now=True)

        return response

    def gripper_force_srv(self,request,response):

        pose = float(request.x)
        force = float(request.y)
        speed = float(request.yaw)
        self.grasp(width=pose,force=force,speed=speed)
        return response

def main(args=None):
    rclpy.init(args=args)

    node = PandaControl()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()