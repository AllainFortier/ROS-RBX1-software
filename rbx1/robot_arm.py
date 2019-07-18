
import Slush
import time
import matplotlib.pyplot as plt
import numpy as np
import config
from controller.data_object import MovementData
from controller.gripper import Gripper


class RobotArm:
    gripper = 7

    def __init__(self, config_file):
        self.board = Slush.sBoard()
        self.loader = config.ConfigLoader()
        self.config = config_file

        # Create and configure joints and gripper from config file
        self.joints = self.loader.create_motors(config_file)
        self.pwm = self.loader.create_gripper(config_file)
        self.gripper = Gripper(self.pwm)

        self.x_history = []
        self.history = []

        self.fig, self.ax = plt.subplots()

        self.tracked_motor = self.motors[3]

    def execute_movement(self, trajectory):
        """

        :param trajectory:
        :return:
        """
        start_time = time.time()
        next_point_dt = 0

        self.x_history = []
        self.history = []

        for point, i in zip(trajectory.points, range(len(trajectory.points))):
            # delay = point.time_from_start.secs + point.time_from_start.nsecs * 1e-9
            time_from_start = time.time() - start_time  # point.time_from_start.secs + point.time_from_start.nsecs * 1e-9

            # See when next point starts
            try:
                next_point_dt = trajectory.points[i+1].time_from_start.secs + trajectory.points[i+1].time_from_start.nsecs * 1e-9
            except IndexError:
                print("No more points, finishing movement")  # No worries, no more points

            print("-----------------------------------------------------------------------------------------------------------------------------------------------------------")
            print(i)
            print(next_point_dt)

            # For all motors, create MovementData object then provides information for movement to be executed
            for motor, pos, vel, acc in zip(self.motors, point.positions, point.velocities, point.accelerations):
                data = MovementData(pos, vel, acc, time_from_start)
                motor.execute_movement(data)

            print(point.accelerations)

            # self.print_debug_position()
            while time.time() - start_time < next_point_dt:
                # Update motor positions, velocity and controllers
                self.update(time.time() - start_time, time.time() - start_time - time_from_start)

                # Record motor position and desired position to plot graph
                self.record(time.time() - start_time,
                            self.tracked_motor.virtual_position,
                            self.tracked_motor.target_position,
                            self.tracked_motor.speed,
                            self.tracked_motor.pid.error)

                # So we don't burn the rpi
                time.sleep(0.025)  # TODO Verify time to sleep

        # Confirms all movement is stopped
        self.soft_stop_arm()

        # Reset controllers before other movements directions
        for mot in self.motors:
            mot.reset()

        print("----------------- Movement completed. {}s ------".format(time.time() - start_time))
        self.print_debug_position()

    def go_positions(self, joint_data):
        for (motor, angle) in zip(self.joints, joint_data.position):
            motor.set_desired_position(motor.convert_rad_to_microsteps(angle))
            motor.move_to_desired()

    def soft_stop_arm(self):
        """
        Uses soft stop on all motors of the arm
        :return: None
        """
        for motor in self.motors:
            motor.softStop()

    def print_debug_position(self):
        targets = []
        currents = []
        diff = []

        for motor in self.motors:
            targets.append(motor.target_position)
            currents.append(motor.physical_position)
            diff.append(motor.target_position - motor.physical_position)

        print('Current: {}'.format(currents))
        print('Target: {}'.format(targets))
        print('Error: {}'.format(diff))

    def update(self, ctime, time):
        for motor in self.joints:
            try:
                motor.update(ctime, time)
            except AttributeError:
                pass  # No movement data object in motor most likely.

    def go_home(self):
        for joint in self.joints:
            joint.goHome()

    def print_status(self):
        for joint in self.joints:
            print(joint)

    def exit(self):
        self.loader.cleanup()

    def plot_motor(self):
        self.ax.plot(self.x_history, self.history)
        plt.show()

    @property
    def motors(self):
        return self.joints

    def wait_for_end_of_movement(self):
        flag = True
        while flag:
            flag = False
            for motor in self.motors:
                motor.update()

                if motor.pid.status:
                    pass
                else:
                    flag = True

            time.sleep(0.15)  # Sleep until the motor is within allowable error.

    def record(self, time, actual_position, target_position, error, velocity):
        self.x_history.append(time)
        self.history.append([actual_position, target_position, error, velocity])

    @property
    def positions(self):
        positions = []
        for i in self.motors:
            positions.append(i.convert_steps_to_rad(i.virtual_position))  # Get position return microsteps so convert
            # positions.append(0)
        return positions

