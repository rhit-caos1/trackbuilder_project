import rclpy
import time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Empty
from movebot_interfaces.srv import AddBox, GetPlanRqst, MoveToTrackRqst
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from franka_msgs.action import Grasp
from franka_msgs.action import Homing
from math import pi
from types import SimpleNamespace

from std_msgs.msg import String


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

        self.timer = self.create_timer(2, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.get_logger().info('Publishing: "%s"' % self.i)
        self.i+=1
    
    def get_track_pose_callback(self, pose_msg):
        # Get track pose (track will get from printer / next track)
        self.track_pose = pose_msg



    def grasp(self, width, speed=1.0, force=30.0, epsilon=(0.005, 0.005)):
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

    async def plan(self,waypoint,execute_now):
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

        await self.plan_client.call_async(self.request)
        # rclpy.spin_until_future_complete(self, future)

        if execute_now and not self.request.goal_pos.orientation :
            await self.send_execute_request()
        elif execute_now and not self.request.goal_pos.position:
            self.get_logger().info(f" FINISHED EXECUTING- CHANGING STATES")
            await self.send_execute_request()
        # return future.result()

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

                    [0.000000,-0.787102,-0.003124,-2.357184,-0.000188,1.571918,0.783990, 0.000323, 0.000323],
                    []
                ],
            "new_home": [

                    [pi/2,-0.783508,-0.000244,-2.356829,-0.003843,1.572944,-0.784056, 0.034865, 0.034865],
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
        await self.plan(self.waypoints.move_test, execute_now=True)
        # self.get_logger().info(f" FINISHED EXECUTING home_robot")
        # self.plan(self.waypoints.rotate_90, execute_now=True)
        # self.get_logger().info(f" FINISHED EXECUTING home_robot")
        return response
    
    async def move_robot(self,request,response):
        await self.plan(self.waypoints.move_test, execute_now=True)
        self.get_logger().info(f" FINISHED EXECUTING move_robot_to_pose")
        return response
    
    def close_gripper_srv(self,request,response):
        self.grasp(width=0.04,force=90.0)
        return response
    
    def open_gripper_srv(self,request,response):
        # self.open_gripper()
        # self.home_gripper()
        # can be controlled by distance and 0 force
        self.grasp(width=0.1,force=00.0)
        return response

    async def move_to_track_srv(self,request,response):
        x = float(request.x)
        y = float(request.y)
        z_0 = 0.486882
        z_1 = 0.0230
        yaw = float(request.yaw)
        await self.plan([[x,y,z_1],[]], execute_now=True)
        self.get_logger().info(f" FINISHED EXECUTING 1")
        time.sleep(3)
        await self.plan([[],[pi, 0.0, yaw]],execute_now=True)
        time.sleep(3)
        await self.plan([[x,y,z_0],[]],execute_now=True)
        self.get_logger().info(f" FINISHED EXECUTING 2")
        # await self.plan([[x,y,z_1],[]],execute_now=True)
        self.grasp(width=0.04,force=90.0)
        time.sleep(3)
        await self.plan(self.waypoints.send_home, execute_now=True)
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