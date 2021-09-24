#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image, Imu
import tf
import numpy as np
import cv2


class RosHandler(object):
    def __init__(self):
        self.encoding = "mono8"
        self.bridge = CvBridge()
        self.image_message = Image()
        self.imu_msgs = Imu()
        self.timestamp = rospy.get_rostime()

        self.image_publisher = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.imu_publisher = rospy.Publisher('/imu', Imu, queue_size=1)

    def publish_image(self, cv_image):
        self.create_ros_image(cv_image)
        self.image_publisher.publish(self.image_message)

        # self.create_ros_camera_calib()
        # self.image_calib_publisher()

    def publish_imu(self, accelerator, gyroscope, orientation):
        self.create_ros_imu_message(accelerator, gyroscope, orientation)
        self.image_publisher.publish(self.imu_msgs)

    def create_ros_imu_message(self, accelerator, gyroscope, orientation):
        orientation = self.degrees2rad(orientation)
        q = tf.transformations.quaternion_from_euler(orientation[0], orientation[1], orientation[3])
        imu = Imu()
        imu.header.frame_id = 'imu'
        imu.header.stamp = self.set_timestamp()
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.linear_acceleration.x = accelerator[0]
        imu.linear_acceleration.y = accelerator[1]
        imu.linear_acceleration.z = accelerator[2]
        imu.angular_velocity.x = gyroscope[0]
        imu.angular_velocity.y = gyroscope[1]
        imu.angular_velocity.z = gyroscope[2]
        self.imu_msgs = imu

    def create_ros_image(self, cv_image):
        gray = cv2.cvtColor(np.array(cv_image), cv2.COLOR_BGR2GRAY)
        image = self.bridge.cv2_to_imgmsg(gray, encoding=self.encoding)
        self.image_message = image
        # self.image_message.header.stamp = self.timestamp
        # self.image_message.data = image
        # print(type(self.image_message.data))
        # print(type(image))
        # self.image_message.header.frame_id = 'mob_camera'
        # self.image_message.encoding = self.encoding

    def set_timestamp(self):
        return rospy.get_rostime()

    @staticmethod
    def degrees2rad(orientation):
        return np.array(orientation) * np.pi / 180

# bridge = CvBridge()
# image_message = bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
# image_message.header.frame_id = camera_frame_id
# calib = CameraInfo()
# calib.header.frame_id = camera_frame_id
# calib.width, calib.height = tuple(util['S_rect_{}'.format(camera_pad)].tolist())
# calib.distortion_model = 'plumb_bob'
# calib.K = util['K_{}'.format(camera_pad)]
# calib.R = util['R_rect_{}'.format(camera_pad)]
# calib.D = util['D_{}'.format(camera_pad)]
# calib.P = util['P_rect_{}'.format(camera_pad)]
