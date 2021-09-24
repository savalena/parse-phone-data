from flask import Flask
from flask_sockets import Sockets
import socket
from ros_handler import RosHandler
import rospy


def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP


rospy.init_node('post_imu_node', anonymous=True)
hostname = socket.gethostname()
IPAddr = get_ip()
print("Your Computer Name is: " + hostname)
print("Your Computer IP Address is: " + IPAddr)
print(
    "Enter {0}:5000 in the app [PhonePi] and select the sensors to stream. For PhonePi+ just enter {0}, without the port".format(
        IPAddr))

app = Flask(__name__)
sockets = Sockets(app)
ros_handler = RosHandler()

@sockets.route('/accelerometer')
def echo_socket(ws):
    while True:
        message = ws.receive()
        print("a:", message)
        ws.send(message)
        splitted = message.split(',')
        print(splitted)
        acc = [float(x) for x in splitted]
        print("aaaaaaaaaaaaa: ", acc)
        ros_handler.update_acceleration(acc)
        ros_handler.publish_imu()


@sockets.route('/gyroscope')
def echo_socket(ws):
    while True:
        message = ws.receive()
        print(message)
        ws.send(message)
        splitted = message.split(',')
        gyro = [float(x) for x in splitted]
        ros_handler.update_gyroscope(gyro)
        ros_handler.publish_imu()


@sockets.route('/orientation')
def echo_socket(ws):
    while True:
        message = ws.receive()
        print(message)
        ws.send(message)
        splitted = message.split(',')
        orientation = [float(x) for x in splitted]
        ros_handler.update_orientation(orientation)
        ros_handler.publish_imu()


@app.route('/')
def hello():
    return 'Hello World!'


if __name__ == "__main__":
    from gevent import pywsgi
    from geventwebsocket.handler import WebSocketHandler

    server = pywsgi.WSGIServer(
        ('0.0.0.0', 5000), app, handler_class=WebSocketHandler)

    server.serve_forever()
    rospy.spin()
