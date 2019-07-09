import math

from Slush import Motor


class MotorController(Motor):

    def __init__(self, motor_number, name):
        Motor.__init__(self, motor_number)

        self.name = name
        self._gear_ratio = 1
        self.index = motor_number
        self.inverted = False

        self.micro_steps = 16
        self.target_position = 0
        self.steps_per_revolution = 200

    def execute_movement(self, velocity, acceleration):
        """
        Will execute a movement at the specified velocity and acceleration
        :param velocity: Steps per seconds
        :param acceleration: Steps per seconds squared
        :return: None
        """
        self.setAccel(abs(acceleration))
        self.setDecel(abs(acceleration))

        if velocity > 0:
            direction = 1
        else:
            direction = 0

        self.run(direction, abs(velocity))

    def execute_movement_radian(self, velocity, acceleration):
        """
        Will execute a movement at the specified velocity and acceleration
        :param velocity: Accepts the velocity in rad/s
        :param acceleration: Accepts the acceleration in rad/s^2
        :return: None
        """
        steps_velocity = self.convert_rad_to_steps(velocity)
        steps_acceleration = self.convert_rad_to_steps(acceleration)

        self.execute_movement(steps_velocity, steps_acceleration)

    def run(self, dir, spd):
        """
        Override the base run method to implement the inverted property.
        :param dir:
        :param spd:
        :return: None
        """
        if self.inverted:
            dir ^= 1

        Motor.run(self, dir, spd)

    def __str__(self):
        return "M:{} Dest:{} |P:{} |V:{}".format(
            self.name, self.target_position, self.getPosition(), self.getSpeed() / 67.106)

    def set_desired_position(self, value):
        if self.inverted:
            self.target_position = -value
        else:
            self.target_position = value

    def move_to_desired(self):
        self.goTo(self.target_position)

    def convert_rad_to_microsteps(self, angle):
        """
        Convert a value in rads to steps for step motor usage
        :param angle: in rads
        :return: n steps
        """
        return int(angle * self.micro_steps_revolution / (2 * math.pi) * self.gear_ratio)

    def convert_rad_to_steps(self, angle):
        """
        Convert a value in rads to steps for step motor usage
        :param angle: in rads
        :return: n steps
        """
        return int(angle * self.steps_per_revolution / (2 * math.pi) * self.gear_ratio)

    def convert_microsteps_to_rad(self, microsteps):
        """
        Convert a value in rads to steps for step motor usage
        :param microsteps: as int of microsteps to convert
        :return: rad angle
        """
        return microsteps / self.gear_ratio * (2 * math.pi) / self.steps_per_revolution

    def convert_steps_to_rad(self, steps):
        """
        Convert a value in rads to steps for step motor usage
        :param steps: as int of steps to convert
        :return: rad angle
        """
        return steps / self.gear_ratio * (2 * math.pi) / self.micro_steps_revolution

    def getPosition(self):
        # if self.inverted:
        #    return -Motor.getPosition(self)
        return Motor.getPosition(self)

    def reset(self):
        pass

    @property
    def micro_steps_revolution(self):
        return self.steps_per_revolution * self.micro_steps

    @property
    def gear_ratio(self):
        return self._gear_ratio

    @gear_ratio.setter
    def gear_ratio(self, ratio):
        self._gear_ratio = ratio

    @property
    def speed(self):
        return self.getSpeed()


class PIDMotorController(MotorController):

    def __init__(self, motor_number, name):
        MotorController.__init__(self, motor_number, name)

    def execute_movement(self, velocity, acceleration):
        """
        Will execute a movement at the specified velocity and acceleration
        :param velocity: Steps per seconds
        :param acceleration: Steps per seconds squared
        :return: None
        """
        self.setAccel(abs(acceleration))
        self.setDecel(abs(acceleration))

        if velocity > 0:
            direction = 1
        else:
            direction = 0

        self.run(direction, abs(velocity))

    def execute_movement_radian(self, velocity, acceleration):
        """
        Will execute a movement at the specified velocity and acceleration
        :param velocity: Accepts the velocity in rad/s
        :param acceleration: Accepts the acceleration in rad/s^2
        :return: None
        """
        steps_velocity = self.convert_rad_to_steps(velocity)
        steps_acceleration = self.convert_rad_to_steps(acceleration)

        self.execute_movement(steps_velocity, steps_acceleration)

    def run(self, dir, spd):
        """
        Override the base run method to implement the inverted property.
        :param dir:
        :param spd:
        :return: None
        """
        if self.inverted:
            dir ^= 1

        Motor.run(self, dir, spd)

    def __str__(self):
        return "M:{} Dest:{} |P:{} |V:{}".format(
            self.name, self.target_position, self.getPosition(), self.getSpeed() / 67.106)


