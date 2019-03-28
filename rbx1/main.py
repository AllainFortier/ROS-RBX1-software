#!/usr/bin/env python3

import RPi.GPIO as GPIO
import json
import queue
from robot_arm import RobotArm
from control_station import ControlStation
from PyQt5.QtWidgets import QApplication


def main():
    try:
        print("============ Initialising Robot Arm software!")

        with open('config.json') as config_file:
            config = json.load(config_file)

        trajectory_queue = queue.Queue()
        gripper_queue = queue.Queue()

        robot_arm = RobotArm(config)

        app = QApplication([])

        station = ControlStation(robot_arm, trajectory_queue, gripper_queue)
        station.show()

        app.exec_()

    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Exiting properly.")
        app.quit()
    finally:
        GPIO.cleanup()


if __name__ == '__main__':
    main()
