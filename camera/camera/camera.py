import rclpy
import time
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import CameraInfo
import cv2
import numpy as np
from cv_bridge import CvBridge 
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from movebot_interfaces.srv import GetCirclesRqst, GetPoseRqst
import time


class Camera(Node):
    
    def __init__(self):
        super().__init__('camera')
        self.camera_matrix = None
        self.distortion_coefficients = None

        self.large_circle_list = []
        self.coarse_positioning = False
        self.fine_positioning = False
        self.detected = False

        self.circle_radius_image_far = 55
        self.circle_radius_image_near = 220
        self.center = None
        self.image = None

        # the list of the images will be capped at 10
        self.image_list = []

        # define subscriber
        self.image_sub = self.create_subscription(
            Image, "camera/color/image_raw", self.get_image_callback, 10
        )
        self.info_camera = self.create_subscription(   
            CameraInfo, "camera/color/camera_info", self.get_info_callback, 10
        )

        # define publisher
        self.result_pub = self.create_publisher(Bool, "result", 10)
        self.image_pub = self.create_publisher(Image, "processed_image", 10)

        # define broadcaster 
        self.tf_broadcaster = TransformBroadcaster(self) 

        # define the service  
        self.circle_service = self.create_service(GetCirclesRqst, "get_circle", self.circle_callback)
        self.pose_service = self.create_service(GetPoseRqst, "get_pose", self.pose_callback)

    def get_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.get_logger().info("Camera matrix: \n" + str(self.camera_matrix))
        self.distortion_coefficients = np.array(msg.d)
        self.get_logger().info("Distortion coefficients: \n" + str(self.distortion_coefficients))
        self.destroy_subscription(self.info_camera)

    def circle_callback(self, request, response):
        self.large_circle_list = []
        self.coarse_positioning = True
        self.fine_positioning = False
        self.detected = False        


        self.get_circle(self.image)   

        x = []
        y = []

        # wait for the large circle list to be filled
        self.get_logger().info("size of the list: " + str(len(self.large_circle_list)))
        # define a vector that points to the center of the circle
        if self.large_circle_list is not None:
            for i in range(len(self.large_circle_list)):
                vector_x = self.large_circle_list[i][0] - self.center[0]
                vector_y = self.large_circle_list[i][1] - self.center[1]
                x.append(vector_x)
                y.append(vector_y)

        response.x = x
        response.y = y
        self.get_logger().info("Send the locations of the circles")   
        return response
    
    def pose_callback(self, request, response):
        self.fine_positioning = True
        self.coarse_positioning = False
        self.get_logger().info("Start pose estimation")

        self.get_circle(self.image)

        response.detected = self.detected
        return response


    def broadcast_transform(self, rvec, tvec):
        try:
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()

            # this frame id will need to match the tag number
            ## TODO
            t.header.frame_id = 'camera_color_optical_frame'
            t.child_frame_id = 'tag_0'

            # convert translation vector to meters from cm
            t.transform.translation.x = float(tvec[0] / 100.0)
            t.transform.translation.y = float(tvec[1] / 100.0)
            t.transform.translation.z = float(tvec[2] / 100.0)

            # convert rotation vector to quaternion
            # convert from (3,1) to (3,)
            rvec = rvec.reshape((3,))
            r = R.from_rotvec(rvec)
            quat = r.as_quat()
            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])
            self.get_logger().info("Quaternion: " + str(quat))

            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().info("Exception: " + str(e))

    def get_image_callback(self, msg):
        if self.camera_matrix is None or self.distortion_coefficients is None:
            self.get_logger().info("No camera matrix or distortion coefficients")
            return
        
        self.image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgra8")
        
        # keep the image list capped at 10
        self.image_list.append(self.image)
        if len(self.image_list) > 10:
            # move everything to the left
            self.image_list = self.image_list[1:]


       

    def get_circle(self, image):
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.center = (image.shape[1]//2, image.shape[0]//2)

        if self.coarse_positioning:

            # use a set of parameters for large circle detection
            circles = cv2.HoughCircles(
                image_gray, cv2.HOUGH_GRADIENT, 1, 50, param1=50, param2=50, minRadius=30, maxRadius= self.circle_radius_image_far)

            # center of the image frame
            

            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                n = 0
                circles_sum = []

                for (x, y, r) in circles:
                    cv2.circle(image, (x, y), r, (0, 255, 0), 4)
                    circles_sum.append([x, y, r])
                    #self.get_logger().info("x: " + str(x) + " y: " + str(y) + " r: " + str(r))
                    
                    already_exist = False
                    for i in range (len(self.large_circle_list)):
                        if (x - self.large_circle_list[i][0])**2 + (y - self.large_circle_list[i][1])**2 < self.circle_radius_image_far**2:
                            already_exist = True
                    if not already_exist:
                        self.large_circle_list.append([x, y])

                    n += 1

        elif self.fine_positioning:

            pose_avg = []
            # for loop for the queue
            for i in range(len(self.image_list)):
                # get the image from the queue
                image = self.image_list[i]
                image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                


                circles = cv2.HoughCircles(
                                image_gray, cv2.HOUGH_GRADIENT, 1.25, 50, param1=40, param2=50, minRadius=20, maxRadius=self.circle_radius_image_near)

                if circles is not None:
                    circles = np.round(circles[0, :]).astype("int")
                    n = 0
                    circles_sum = []

                    for (x, y, r) in circles:
                        cv2.circle(image, (x, y), r, (0, 255, 0), 4)

                        #check to see if the circle is around the center of the image
                        if (x - self.center[0])**2 + (y - self.center[1])**2 > (self.circle_radius_image_near + 15)**2:
                            continue

                        circles_sum.append([x, y, r])
                        n += 1
                    circles_sum = sorted(circles_sum, key=lambda x: x[2])

                    #put text on the circle
                    for i in range(len(circles_sum)):
                        cv2.putText(image, str(i), (circles_sum[i][0], circles_sum[i][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

                
                    if len(circles_sum) == 4:
                        #self.get_logger().info(str(len(circles_sum[0])))
                        # still need to implement a way to check if the circles are in the correct radius
                        points_2D = np.array([(circles_sum[3][0], circles_sum[3][1]),
                                            (circles_sum[2][0], circles_sum[2][1]),
                                                (circles_sum[1][0], circles_sum[1][1]),
                                            (circles_sum[0][0], circles_sum[0][1])], dtype="double")
                        points_3D = np.array([(0.0, 0.0, 0.0),
                                            (1.0, 0.0, 0.0),
                                            (-0.8, -1.2, 0.0),
                                            (-0.8, 1.2, 0.0)])
                        success,rvec,tvec = cv2.solvePnP(points_3D, points_2D, self.camera_matrix, self.distortion_coefficients)
                        pose_avg.append([rvec, tvec])
                    else:
                        self.get_logger().info("Not enough circles detected")
                    
                else:
                    self.get_logger().info("No tag detected")

            # calculate the average of the rvec and tvec
            rvec = np.mean(np.array(pose_avg)[:, 0], axis=0)
            tvec = np.mean(np.array(pose_avg)[:, 1], axis=0)
            image = cv2.drawFrameAxes(image, self.camera_matrix, self.distortion_coefficients, rvec, tvec, 2.5)

            self.get_logger().info("Pose estimation successful")
            self.detected = True

            # broadcast transform
            self.broadcast_transform(rvec, tvec)

        #publish the image
        self.image_pub.publish(CvBridge().cv2_to_imgmsg(image, encoding="bgra8"))



def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()