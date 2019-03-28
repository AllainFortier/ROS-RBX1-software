
import Slush
import time
import config
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

    def execute_movement(self, trajectory):
        """

        :param trajectory:
        :return:
        """
        start_time = time.time()

        for point in trajectory.points:
            for motor, pos, vel, acc in zip(self.motors, point.positions, point.velocities, point.accelerations):
                motor.set_desired_position(motor.convert_rad_to_microsteps(pos))
                motor.execute_movement_radian(vel, acc)

            delay = point.time_from_start.secs + point.time_from_start.nsecs * 1e-9
            while time.time() - start_time < delay:
                continue  # Poll while we wait for time to pass.

        self.print_debug_position()

        # Finalise the movements by moving to the exact desired position
        for motor in self.motors:
            motor.move_to_desired()

        print("----------------- Movement completed. {}s ------".format(time.time() - start_time))
        self.wait_for_end_of_movement()
        self.print_debug_position()

    def go_positions(self, joint_data):
        for (motor, angle) in zip(self.joints, joint_data.position):
            motor.goTo(motor.convert_rad_to_microsteps(angle))

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
            currents.append(motor.getPosition())
            diff.append(motor.target_position - motor.getPosition())

        print('Current: {}'.format(currents))
        print('Target: {}'.format(targets))
        print('Error: {}'.format(diff))

    def update(self):
        for motor in self.joints:
            motor.update()

    def go_home(self):
        for joint in self.joints:
            joint.goHome()

    def print_status(self):
        for joint in self.joints:
            print(joint)

    def exit(self):
        self.loader.cleanup()

    @property
    def motors(self):
        return self.joints

    def wait_for_end_of_movement(self):
        for motor in self.motors:
            motor.waitMoveFinish()

