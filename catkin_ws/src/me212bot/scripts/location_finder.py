import cv2
import pyrealsense2
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad
from sensor_msgs.msg import CameraInfo
from apriltags.msg import AprilTagDetections
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np

table_id = 8

class WorldFrame:
    def __init__(self, gripper_offset):
        self.gripper_offset = gripper_offset  # vector [X,Y,Z] from the UR5 endpoint to the end-effector point in meters
        apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, apriltag_callback, queue_size=1)
        self.cam_topic = rospy.get_param("~cam_topic", "/camera/color/image_raw")
        self.cam_sub = rospy.Subscriber(self.cam_topic, Image, self.get_coords())
        self.cam_info = rospy.Subscriber("/camera/cameraInfo")

    def pixel2coord(self, x, y, depth, cameraInfo):
        """Get the x,y,z coordinates of a pixel location relative to the camera location"""

        _intrinsics = pyrealsense2.intrinsics()
        _intrinsics.width = cameraInfo.width
        _intrinsics.height = cameraInfo.height
        _intrinsics.ppx = cameraInfo.K[2]
        _intrinsics.ppy = cameraInfo.K[5]
        _intrinsics.fx = cameraInfo.K[0]
        _intrinsics.fy = cameraInfo.K[4]
        # _intrinsics.model = cameraInfo.distortion_model
        _intrinsics.model = pyrealsense2.distortion.none
        _intrinsics.coeffs = [i for i in cameraInfo.D]
        result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
        # result[0]: right, result[1]: down, result[2]: forward
        return result[2], -result[0], -result[1]

    def initialize_camera(self, image):
        """Get the x,y,z coordinates of the april tag relative to the camera"""


    def get_coords(self):


def apriltag_callback(data):
    # use apriltag pose detection to find where is the robot
    for detection in data.detections:
        if detection.id == table_id:  # tag id is the correct one
            pass