#!/usr/bin/env python

import asyncio
import websockets
import socket
from ros_handler import RosHandler
import rospy
import argparse


class ImuData(RosHandler):
    def __init__(self):
        super(RosHandler, self).__init__()
        self.hostname = socket.gethostname()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ip = self.get_ip()
        print("Your Computer Name is: " + self.hostname)
        print("Your Computer IP Address is: " + self.ip)
        print(
            "Enter {0}:5000 in the app [PhonePi] and select the sensors to stream. For PhonePi+ just enter {0}, without the port".format(
                self.ip))
        asyncio.get_event_loop().run_until_complete(
            websockets.serve(self.echo, '0.0.0.0', 5000))
        asyncio.get_event_loop().run_forever()

    def get_ip(self):
        try:
            # doesn't even have to be reachable
            self.socket.connect(('10.255.255.255', 1))
            IP = self.socket.getsockname()[0]
        except Exception:
            IP = '127.0.0.1'
        finally:
            self.socket.close()
        return IP

    async def echo(self, websocket, path):
        async for message in websocket:
            if path == '//accelerometer':
                data = await websocket.recv()
                splitted = data.split(',')
                self.acceleration = [float(x) for x in splitted]

            if path == '//gyroscope':
                data = await websocket.recv()
                splitted = data.split(',')
                self.gyroscope = [float(x) for x in splitted]

            if path == '//orientation':
                data = await websocket.recv()
                splitted = data.split(',')
                self.orientation = [float(x) for x in splitted]

            self.publish_imu(self.acceleration, self.gyroscope, self.orientation)


# hostname = socket.gethostname()
# IPAddr = get_ip()
# print("Your Computer Name is: " + hostname)
# print("Your Computer IP Address is: " + IPAddr)
# print(
#     "Enter {0}:5000 in the app [PhonePi] and select the sensors to stream. For PhonePi+ just enter {0}, without the port".format(
#         IPAddr))


# async def echo(websocket, path):
#     async for message in websocket:
#         if path == '//accelerometer':
#             data = await websocket.recv()
#             splitted = data.split(',')
#             acc = [float(x) for x in splitted]
#             print("accelerator ", data)
#             print("type ", type(data))
#
#         if path == '//gyroscope':
#             data = await websocket.recv()
#             print('gyro ', data)
#
#         if path == '//orientation':
#             data = await websocket.recv()
#             print(data)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test')
    parser.add_argument('--test', type=bool, help='Test camera connection ?', default=True)
    args = parser.parse_args()

    rospy.init_node('post_image_node', anonymous=True)
    try:
        imu = ImuData()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass