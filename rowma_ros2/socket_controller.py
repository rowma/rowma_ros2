import os
import ast
import time
import json
import sys

from subprocess import Popen, PIPE
from . import utils
from .version import version

class SocketController:
    def __init__(self, node, id, subscribers, sio, nms):
        self.node = node
        self.id = id
        self.subscribers = subscribers
        self.sio = sio
        self.nms = nms
        self.reconnecting = False

    def connect(self):
        launch_commands = utils.list_launch_commands()
        rosrun_commands = utils.list_rosorun_commands()

        uuid = os.environ.get('UUID', self.id)
        rostopics = list(map(lambda x: x[0], self.node.get_topic_names_and_types()))
        msg = {
                'uuid': uuid,
                'launch_commands': launch_commands,
                'rosnodes': self.node.get_node_names(),
                'rosrun_commands': rosrun_commands,
                'rowma_ros_version': version,
                'rostopics': rostopics,
                'reconnection': self.reconnecting
                }

        api_key = os.environ.get('API_KEY')
        if api_key:
            msg['api_key'] = api_key

        self.sio.emit('register_robot', json.dumps(msg), namespace=self.nms)
        utils.print_success('connection established')

    def robot_registered(self, data):
        self.id = self.id or data['uuid']
        utils.print_success('Your UUID is: ' + self.id)

    def signal_handler(self):
        self.sio.disconnect()
        sys.exit(0)

    def outgoing_func(self, message):
        if len(self.subscribers) == 0:
            return
        destinations = []
        msg = json.loads(message)
        for subscriber in self.subscribers:
            if subscriber['topic'] == msg['topic']:
                destination = subscriber['destination']

        if destination:
            msg['topicDestination'] = destination
            msg['sourceUuid'] = self.id
            self.sio.emit('topic_from_ros', json.dumps(msg), namespace=self.nms)

    def set_reconnecting(self, reconnecting):
        self.reconnecting = reconnecting

    def rostopic(self, data, protocol):
        # TODO: Separate by operation
        if data['op'] == 'subscribe':
            newSubscriber = { 'topic': data['topic'], 'destination': data['topicDestination'] }
            self.subscribers.append(newSubscriber)
        message = ast.literal_eval(json.dumps(data))
        protocol.incoming(json.dumps(message))

    def run_launch(self, data):
        launch_commands = utils.list_launch_commands()
        if data.get('command') in launch_commands:
            cmd = 'PYTHONUNBUFFERED=false ros2 launch ' + data.get('command')
            process = Popen(cmd, shell=True, stdout=PIPE)

            # Note: The launched rosnode-name does not appear the soon after roslaunch is executed.
            # Therefore, sleep is neccessary to wait it finishes to launch.
            time.sleep(2)
            rostopics = list(map(lambda x: x[0], self.node.get_topic_names_and_types()))
            msg = {
                'uuid': self.id,
                'rosnodes': self.node.get_node_names(),
                'rostopics': rostopics
                }
            self.sio.emit('update_rosnodes', json.dumps(msg), namespace=self.nms)

            while True:
                output = process.stdout.readline()
                if output == '' and process.poll() is not None:
                    break
                if output:
                    msg = {
                            "cmd": data.get('command').replace(" ", "-"),
                            "robotUuid": self.id,
                            "log": output.strip(),
                          }
                    self.sio.emit('roslaunch_log', json.dumps(msg), namespace=self.nms)
                    print(output.strip())
            process.poll()

    def run_rosrun(self, data):
    	rosrun_commands = utils.list_rosorun_commands()
    	if data.get('command') in rosrun_commands:
            cmd = 'PYTHONUNBUFFERED=false ros2 run ' + data.get('command') + ' ' + data.get('args')
            process = Popen(cmd, shell=True, stdout=PIPE)

            # Note: The launched rosnode-name does not appear the soon after roslaunch is executed.
            # Therefore, sleep is neccessary to wait it finishes to launch.
            time.sleep(2)
            rostopics = list(map(lambda x: x[0], self.node.get_topic_names_and_types()))
            msg = {
                'uuid': self.id,
                'rosnodes': self.node.get_node_names(),
                'rostopics': rostopics
                }
            self.sio.emit('update_rosnodes', json.dumps(msg), namespace=self.nms)

            while True:
                output = process.stdout.readline()
                if output == '' and process.poll() is not None:
                    break
                if output:
                    msg = {
                            "cmd": data.get('command').replace(" ", "-"),
                            "robotUuid": self.id,
                            "log": output.strip(),
                          }
                    self.sio.emit('rosrun_log', json.dumps(msg), namespace=self.nms)
                    print(output.strip())
            process.poll()

    def kill_rosnodes(self, data):
        cmd = 'ros2 lifecycle set ' + data.get('rosnodes') + ' shutdown'
        sp.check_output(cmd, shell=True).decode('utf-8').strip().split('\n')
        # Note: The launched rosnode-name does not appear the soon after roslaunch is executed.
        # Therefore, sleep is neccessary to wait it finishes to launch.
        time.sleep(2)
        rostopics = list(map(lambda x: x[0], self.node.get_topic_names_and_types()))
        msg = {
            'uuid': self.id,
            'rosnodes': self.rosnode.get_node_names(),
            'rostopics': rostopics
            }
        self.sio.emit('update_rosnodes', json.dumps(msg), namespace=self.nms)
        print('killed')

    def unsubscribe_rostopic(self, data):
        utils.print_debug(data)
        self.subscribers = list(filter(lambda s: s['topic'] != data.get('topic'), self.subscribers))
        rostopics = list(map(lambda x: x[0], self.node.get_topic_names_and_types()))
        msg = {
            'uuid': self.id,
            'rosnodes': self.node.get_node_names(),
            'rostopics': rostopics
            }
        self.sio.emit('update_rosnodes', json.dumps(msg), namespace=self.nms)

    # def add_script(self, data):
    #     enable_script_download = os.environ.get('ENABLE_SCRIPT_DOWNLOAD', False)
    #     if enable_script_download:
    #         script = data.get('script')
    #         name = data.get('name')
    #         current_path = os.path.dirname(os.path.realpath(__file__))
    #         file_path = os.path.join(current_path, "../" + name)
    #         file = open(file_path, 'w')
    #         file.write(script)
    #         file.close()

    #         cmd = 'chmod +x ' + file_path
    #         process = Popen(cmd, shell=True, stdout=PIPE)

    # 	    msg = {
    # 	        'uuid': self.id,
    #             'rosrunCommands': utils.list_rosorun_commands()
    # 	        }
    # 	    self.sio.emit('update_rosnodes', json.dumps(msg), namespace=self.nms)
    #     else:
    #         utils.print_error("You have to set ENABLE_SCRIPT_DOWNLOAD if you use script download.")
