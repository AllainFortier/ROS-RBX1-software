#!/usr/bin/env python

import RPi.GPIO as GPIO
import json
import Queue
from Queue import Queue
from robot_arm import RobotArm
from control_station import ControlStation
from PyQt5.QtWidgets import QApplication

from threads.joint_state_publisher import JointStatePublisher


def main():
    try:
        print("============ Initialising Robot Arm software!")

        with open('config.json') as config_file:
            config = json.load(config_file)

        trajectory_queue = Queue()
        gripper_queue = Queue()

        # Initialises the arm with the desired configuration
        robot_arm = RobotArm(config)

        # Creates the thread that will continuously publish the state of the robot to ROS
        joint_publisher = JointStatePublisher(robot_arm)
        joint_publisher.start()

        # Runs the UI to manually control the arm directly through SPI # TODO Run as service with no UI
        app = QApplication([])
        station = ControlStation(robot_arm, trajectory_queue, gripper_queue)
        station.show()

        app.exec_()

        # Cleanup methods after GUI exits
        joint_publisher.stop()
        joint_publisher.thread.join(timeout=5)

    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Exiting properly.")
        app.quit()
    finally:
        GPIO.cleanup()


if __name__ == '__main__':
    main()
