#!/usr/bin/env python
import cv2
import rospy
from ros_handler import RosHandler
import argparse
import numpy as np


# 'http://10.16.100.83:4747/video?640x480'
class ImageIpCam(RosHandler):
    def __init__(self, test, camera_ip='http://10.16.100.21:4747/video?640x480'):
        super(ImageIpCam, self).__init__()
        self.cap = cv2.VideoCapture(camera_ip)
        self.get()

    def get(self):
        while True:
            ret, frame = self.cap.read()
            if frame is not None:
                self.publish_image(frame)
            else:
                rospy.logwarn("Image haven't arrived or camera is not connected")
                rospy.sleep(1)

    def test_connections(self):
        frame = []
        while True:
            rospy.sleep(1)
            ret, frame = self.cap.read()
            if frame != []:
                cv2.imshow("Capturing", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    return
            else:
                rospy.logwarn("Image haven't arrived or camera is not connected")
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test')
    parser.add_argument('--test', type=bool, help='Test camera connection ?', default=True)
    args = parser.parse_args()

    rospy.init_node('post_image_node', anonymous=True)
    try:
        camera_handler = ImageIpCam(args.test)
        rospy.spin()
        camera_handler.cap.release()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass
