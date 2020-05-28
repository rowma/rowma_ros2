import rclpy
from rclpy.node import Node
import socketio
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
import signal
import os
import sys
from .socket_controller import SocketController
from . import utils

from std_msgs.msg import String

sio = socketio.Client(
    reconnection=True,
    reconnection_attempts=0,
    reconnection_delay=1,
    reconnection_delay_max=30
)

if os.environ.get('API_KEY'):
    nms = '/rowma_robot'
else:
    nms = '/rowma'

rclpy.init()
rowma = Node('rowma')
sc = SocketController(rowma, '', [], sio, nms)

@sio.event(namespace=nms)
def connect():
    sc.connect()
    sc.set_reconnecting(False)

@sio.on('robot_registered', namespace=nms)
def on_message(data):
    sc.robot_registered(data)

@sio.on('err', namespace=nms)
def on_message(data):
    utils.print_error(data.get('error'))
    # TODO: More smart exit
    os._exit(1)

@sio.on('rostopic', namespace=nms)
def on_message(data):
    sc.rostopic(data, protocol)

@sio.on('run_launch', namespace=nms)
def on_message(data):
    sc.run_launch(data)

@sio.on('run_rosrun', namespace=nms)
def on_message(data):
    sc.run_rosrun(data)

@sio.on('kill_rosnodes', namespace=nms)
def on_message(data):
    sc.kill_rosnodes(data)

@sio.on('unsubscribe_rostopic', namespace=nms)
def on_message(data):
    sc.unsubscribe_rostopic(data)

# @sio.on('add_script', namespace=nms)
# def on_message(data):
#     sc.add_script(data)

@sio.event(namespace=nms)
def disconnect():
    sc.set_reconnecting(True)

def outgoing_func(message):
    sc.outgoing_func(message)

def signal_handler(sig, frame):
    sys.exit(0)

def main(args=None):
    server_url = os.environ.get('ROWMA_SERVER_URL', 'https://rowma.moriokalab.com')
    sio.connect(server_url)

    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()

    client_id_seed = 0
    protocol = RosbridgeProtocol(client_id_seed, rowma)

    protocol.outgoing = outgoing_func

    rclpy.spin(rowma)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rowma.destroy_node()
    rclpy.shutdown()
