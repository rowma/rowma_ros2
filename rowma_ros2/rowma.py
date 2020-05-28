import rclpy
from rclpy.node import Node
import socketio
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
import signal
import os
import sys

from std_msgs.msg import String

class Rowma(Node):
    def __init__(self):
        super().__init__('rowma')

def outgoing_func(message):
    sc.outgoing_func(message)

def signal_handler(sig, frame):
    sys.exit(0)

def main(args=None):
    sio = socketio.Client(
        reconnection=True,
        reconnection_attempts=0,
        reconnection_delay=1,
        reconnection_delay_max=30
    )

    server_url = os.environ.get('ROWMA_SERVER_URL', 'https://rowma.moriokalab.com')
    sio.connect(server_url)

    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()

    rclpy.init(args=args)

    rowma = Node('rowma')
    client_id_seed = 0
    protocol = RosbridgeProtocol(client_id_seed, rowma)

    protocol.outgoing = outgoing_func

    rclpy.spin(rowma)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rowma.destroy_node()
    rclpy.shutdown()
