import rclpy
import math
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from movebot_interfaces.srv import ScanPoints

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


class TagTF(Node):

    def __init__(self):
        super().__init__('tag_tf')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tag_detect_srv = self.create_service(ScanPoints, "tag_detect", self.tag_detect_srv)

        self.timer= self.create_timer(2, self.timer_callback)
        self.tag_x = 0
        self.tag_y = 0
        self.tag_az = 0

    def timer_callback(self):
        try:
            tag_2_base = self.tf_buffer.lookup_transform(
                'panda_link0',
                'tag_0',
                rclpy.time.Time())
            self.tag_x = tag_2_base.transform.translation.x
            self.tag_y = tag_2_base.transform.translation.y
            self.get_logger().info(f"tag_x: {self.tag_x}, tag_y: {self.tag_y}")
            # tag_ax,tag_ay,self.tag_az = euler_from_quaternion(
            #     tag_2_base.transform.rotation.x,
            #     tag_2_base.transform.rotation.y,
            #     tag_2_base.transform.rotation.z,
            #     tag_2_base.transform.rotation.w,
            # )

        except Exception as e:
            self.get_logger().info("unable to find tf")
            self.get_logger().info(f"error: {e}")

        self.get_logger().info("timer_tf")

    def tag_detect_srv(self,request,response):
        try:
            tag_2_base = self.tf_buffer.lookup_transform(
                'panda_link0',
                'tag_0',
                rclpy.time.Time())
            self.tag_x = tag_2_base.transform.translation.x
            self.tag_y = tag_2_base.transform.translation.y
            self.get_logger().info(f"tag_x: {self.tag_x}, tag_y: {self.tag_y}")
            tag_ax,tag_ay,self.tag_az = euler_from_quaternion(
                tag_2_base.transform.rotation.x,
                tag_2_base.transform.rotation.y,
                tag_2_base.transform.rotation.z,
                tag_2_base.transform.rotation.w,
            )
            response.x = self.tag_x
            response.y = self.tag_y
            response.theta = self.tag_az
            response.success = True

        except Exception as e:
            self.get_logger().info("unable to find tf")
            self.get_logger().info(f"error: {e}")
            response.success = False
        
        return response



def main(args=None):
    rclpy.init(args=args)

    node = TagTF()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # node.destroy_node()
    rclpy.shutdown()
