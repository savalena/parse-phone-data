#!/usr/bin/env python3

from ros_handler import RosHandler
import rospy
import asyncio
import websockets
import socket


class ImuData(RosHandler):
    def __init__(self):
        super(ImuData, self).__init__()
        self.hostname = socket.gethostname()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ip = self.get_ip()
        self.acceleration = []
        self.gyroscope = []
        self.orientation = []
        print("Your Computer Name is: " + self.hostname)
        print("Your Computer IP Address is: " + self.ip)
        print(
            "Enter {0}:5000 in the app [PhonePi] and select the sensors to stream. For PhonePi+ just enter {0}, without the port".format(
                self.ip))

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

    def check_params(self):
        if self.acceleration != [] and self.gyroscope != [] and self.orientation != []:
            return True
        else:
            return False

    async def echo(self, websocket, path):
        async for message in websocket:
            if path == '//accelerometer':
                data = await websocket.recv()
                print("acceleration")
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

            print('a', self.acceleration)
            print('g', self.gyroscope)
            print('o', self.orientation)
            if self.check_params():
                print("INSIDE----------------------------------")
                self.publish_imu(self.acceleration, self.gyroscope, self.orientation)


if __name__ == '__main__':
    global acceleration, gyroscope, orientation
    rospy.init_node('post_imu_node', anonymous=True)
    ros_handler = RosHandler()
    imu = ImuData()

    asyncio.get_event_loop().run_until_complete(websockets.serve(imu.echo, '0.0.0.0', 5000))
    asyncio.get_event_loop().run_forever()
    rospy.spin()
