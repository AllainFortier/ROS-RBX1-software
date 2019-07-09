#!/usr/bin/env python

"""
Really, only purpose of the bridge is to take information from Python 2.7 where ROS runs to Slush on Python 3
"""
import os
import time
import rospy
import sensor_msgs.msg
import zmq
from control_msgs.msg import FollowJointTrajectoryAction, GripperCommandAction, FollowJointTrajectoryFeedback
from moveit_msgs.msg import PlaceAction, PickupAction
import actionlib
from std_msgs.msg import Header


class Bridge:

    _trajectory_topic = "/rbx1_arm_controller/follow_joint_trajectory"
    _goal_server_topic = "/gripper_controller/gripper_action"

    def __init__(self):
        rospy.init_node('rbx1_arm_controller', anonymous=True)

        try:
            os.makedirs('/tmp/feeds/')
        except OSError:
            print('Directory /tmp/feeds/ already exist.')

        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.PUB)
        self._protocol = 'ipc'
        self._path = '/tmp/feeds/rbx'

        self._trajectory_server = actionlib.SimpleActionServer(self._trajectory_topic, FollowJointTrajectoryAction,
                                                               execute_cb=self._follow_joint_trajectory_callback,
                                                               auto_start=False)

        self._gripper_server = actionlib.SimpleActionServer(self._goal_server_topic, GripperCommandAction,
                                                            execute_cb=self._gripper_callback,
                                                            auto_start=False)

        # self._place_server = actionlib.SimpleActionServer("/place", PlaceAction,
        #                                                   execute_cb=self.place_cb, auto_start=True)

        # self._place_server = actionlib.SimpleActionServer("/pickup", PickupAction,
        #                                                  execute_cb=self.place_cb, auto_start=True)

        self._trajectory_server.start()
        self._gripper_server.start()

    def connect(self):
        self._socket.bind("{}://{}".format(self._protocol, self._path))

    def _follow_joint_trajectory_callback(self, data):
        print("Trajectory information received")
        topic = "arm"
        self._socket.send_string(topic, zmq.SNDMORE)
        self.publish_zmq(data)

        self._trajectory_server.set_succeeded()  # TODO better handle

    def _gripper_callback(self, data):
        print("Gripper command received")
        topic = "gripper"
        self._socket.send_string(topic, zmq.SNDMORE)
        self.publish_zmq(data)

        self._gripper_server.set_succeeded()  # TODO better handle

    def place_cb(self):
        pass

    def publish_zmq(self, data):
        self._socket.send_pyobj(data)


if __name__ == '__main__':
    bridge = Bridge()
    bridge.connect()

    rospy.spin()
