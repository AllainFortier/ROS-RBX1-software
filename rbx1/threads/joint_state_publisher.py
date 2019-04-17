
from threading import Thread

import rospy
import sensor_msgs.msg
from std_msgs.msg import Header

from robot_arm import RobotArm


class JointStatePublisher:

    attached_arm = None

    def __init__(self, robot_arm):
        # type: (RobotArm) -> None
        """

        :param robot_arm:
        """

        rospy.init_node('RbxJointStatePublisher')
        self.rate = rospy.Rate(5)

        self.joint_state_publisher = rospy.Publisher('/joint_states', sensor_msgs.msg.JointState, queue_size=20)

        self.joint_names = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Joint_6', 'Joint_Grip_Servo']
                                        #,'Joint_Tip_Servo', 'Joint_Grip_Servo_Arm', 'Joint_Grip_Idle','Joint_Tip_Idle', 'Joint_Grip_Idle_Arm']
        self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.robot_arm = robot_arm
        self.running = False
        self.thread = Thread(target=self._start)

    def start(self):
        """
        Starts the thread that will execute the task
        :return:
        """
        self.thread.start()

    def stop(self):
        """
        Stops the thread that will execute the task
        :return:
        """
        self.running = False

    def _start(self):
        """
        Starts the task
        :return:
        """
        self.running = True

        rospy.loginfo("Starting publishing to /joint_states")

        try:
            while self.running:
                self.update_position()
                self.execute_task()
                self.rate.sleep()

        except AttributeError as e:
            print("Attribute error: {}".format(e))  # TODO better

    def update_position(self):
        """
        Take the position from the Robot Arm and update our own position register
        :return: None
        """
        self.position = self.robot_arm.positions

    def execute_task(self):
        """
        When receiving info from ROS, put it on queue and signal that work is available.
        :return: None
        """

        joint_states = sensor_msgs.msg.JointState()
        joint_states.header = Header()
        joint_states.header.stamp = rospy.Time.now()
        joint_states.name = self.joint_names
        joint_states.position = self.position
        joint_states.velocity = []
        joint_states.effort = []

        self.joint_state_publisher.publish(joint_states)


if __name__ == '__main__':
    try:
        rospy.init_node('CustomJointStatePublisher')
        pub = JointStatePublisher(None)
        pub.start()

    except KeyboardInterrupt:
        print("Exiting app.")