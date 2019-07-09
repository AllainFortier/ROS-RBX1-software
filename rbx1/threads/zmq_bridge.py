import os

import zmq
from PyQt5 import QtCore
from PyQt5.QtCore import pyqtSignal, pyqtSlot


class ReceiverBridge(QtCore.QObject):

    signal_data_received = pyqtSignal(str)
    _attached_arm = None

    def __init__(self, sync_queue, topic='', parent=None):
        QtCore.QObject.__init__(self, parent)

        try:
            os.makedirs('/tmp/feeds/')
        except OSError:
            print('Info: Directory /tmp/feeds/ already exist.')

        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._protocol = 'ipc'
        self._path = '/tmp/feeds/rbx'

        self.topic = topic

        self._socket.connect("{}://{}".format(self._protocol, self._path))

        self.running = False

        self.queue = sync_queue
        self.thread = None

    @pyqtSlot()
    def start(self):
        """

        :return:
        """
        self.running = True

        print("Starting monitoring topic {}".format(self.topic))
        self._socket.subscribe(self.topic)

        while self.running:
            self.execute_task()

    def execute_task(self):
        """
        When receiving info from ROS, put it on queue and signal that work is available.
        :return: None
        """

        try:
            topic = self._socket.recv_string()
            obj = self._socket.recv_pyobj()

            # put obj on task queue then send signal that work is to be completed
            self.queue.put(obj)
            self.signal_data_received.emit("Data received")

        except AttributeError as e:
            print("Attribute error: {}".format(e))  # TODO better
        except zmq.Again:
            print("Again error")
        except zmq.ZMQError:
            print("No Block")

    def quit(self):
        self.running = False