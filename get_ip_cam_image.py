#!/usr/bin/env python
import cv2
import rospy
from ros_handler import RosHandler
import argparse


# 'http://10.16.100.83:4747/video?640x480'
class ImageIpCam(RosHandler):
    def __init__(self, test, camera_ip='http://10.16.100.83:4747/video?640x480'):
        super(RosHandler, self).__init__()
        self.cap = cv2.VideoCapture(camera_ip)
        if test:
            print('******************CONNECTION WITH CAMERA TEST*********************')
            self.test_connections()
        else:
            print('******************STARTING POST IMAGES IN ROS*********************')
            self.get()

    def get(self):
        while True:
            _, frame = self.cap.read()
            self.publish_image(frame)

    def test_connections(self):
        for i in range(100):
            ret, frame = self.cap.read()
            cv2.imshow("Capturing", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

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
