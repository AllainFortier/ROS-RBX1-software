import math

from pprint import pprint

from Slush import Motor

from controller.PIDController import PIDController
from controller.data_object import MovementData


class MotorController(Motor):

    def __init__(self, motor_number, name):
        Motor.__init__(self, motor_number)

        self.name = name
        self._gear_ratio = 1
        self._index = motor_number
        self.inverted = False

        self.micro_steps = 16
        self.target_position = 0
        self.steps_per_revolution = 200
        self._desired_velocity = 0
        self.direction = 0

        self.speed_average = []
        self.position_average = []

        self.movement_data = None  # type: MovementData

    def execute_movement(self, data):
        """
        Will execute a movement at the specified velocity and acceleration
        :param position:
        :param dt:
        :param velocity: Steps per seconds
        :param acceleration: Steps per seconds squared
        :return: None
        """
        self.movement_data = data
        velocity_steps = self.convert_rad_to_steps(data.velocity)

        if data.velocity > 0:
            self.direction = 1
        else:
            self.direction = 0

        self.run(self.direction, abs(velocity_steps))

    def run(self, dir, spd):
        """
        Override the base run method to implement the inverted property.
        :param dir:
        :param spd:
        :return: None
        """
        Motor.run(self, dir, spd)

    def __str__(self):
        return "M:{} Dest:{} |P:{} |V:{} ({})".format(
            self.name, self.target_position, self.virtual_position, self.getSpeed() / 67.106, self.inverted)

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

    @property
    def physical_position(self):
        """
        Returns the physical position of the motor unaffected by virtual inverted properties.
        :return:
        """
        return Motor.getPosition(self)

    @property
    def virtual_position(self):
        """
        Returns the virtual position affected by whether the motor is set to be inverted or not.
        :return:
        """
        if self.inverted:
            return -Motor.getPosition(self)
        return Motor.getPosition(self)

    def getPosition(self):
        raise AttributeError("Do not use method for subclasses.")

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
    def index(self):
        return self._index

    @property
    def speed(self):
        speed = self.getSpeed()
        return speed


class PIDMotorController(MotorController):

    def __init__(self, motor_number, name):
        MotorController.__init__(self, motor_number, name)

        self.pid = PIDController()
        self.target_velocity = 0

        self._debug_motor_monitor = 3

    def execute_movement(self, data):
        """
        Will execute a movement at the specified velocity and acceleration.
        All values in radians.
        :param data: 
        :return: None
        """
        self.movement_data = data

        if self.index == self._debug_motor_monitor:
            print("{}".format(str(data.position)))

        self.set_desired_position(self.convert_rad_to_microsteps(data.position))
        self.pid.target_value = self.target_position

    def update(self, ctime, dt):
        """
        Method to update the PID feedback values
        :return: None
        """
        # Taking velocity and acceleration values, compute the current desired position.
        adjusted_position = self.get_interpolation_position(dt,
                                                            self.movement_data.position,
                                                            self.movement_data.velocity,
                                                            self.movement_data.acceleration)

        self.set_desired_position(self.convert_rad_to_microsteps(adjusted_position))
        self.pid.target_value = self.target_position

        self.pid.update(self.physical_position)

        # Set motor velocity from PID output
        self.target_velocity += self.pid.output
        self.set_motor_desired_velocity(self.target_velocity)  # PID output is the speed diff

        # Make the physical motor spin. Convert rads to steps/s at the latest minute.
        self.run(self.direction, abs(self.target_velocity))

        if self.index == self._debug_motor_monitor:
            print("ctime {:.2f}s | dt {:.2f}s".format(ctime, dt))
            print("{:.2f} | 0.5*{:.2f}*x**2 + {:.2f}*x + {:.2f}".format(adjusted_position,
                                                                        self.movement_data.position,
                                                                        self.movement_data.velocity,
                                                                        self.movement_data.acceleration))
            print("{} | {} | pos{:.2f}".format(self.name, str(self.pid), self.virtual_position))
            print("curr speed {:.2f} | target speed {:.2f}".format(self.speed, self.target_velocity))

    def get_interpolation_position(self, time, start_position, velocity, acceleration):
        """
        Using the 1/2ax**2 + bx + x with x being in seconds, we find the desired position needed.
        :param time: in seconds
        :param start_position: in steps
        :param velocity: in steps/s
        :param acceleration: in steps/s**2
        :return: The position where we should be at the defined time and position
        """
        position = (acceleration * time ** 2) / 2 + (velocity * time) + start_position
        return position

    def set_motor_desired_velocity(self, velocity):
        """
        Convenience method that adjust the turning direction of the motor depending on the velocity sign
        then sets the target velocity.
        :param velocity: in steps per seconds
        :return:
        """
        if velocity > 0:
            self.direction = 1
        else:
            self.direction = 0

        self.target_velocity = velocity

    def reset(self):
        self.pid.reset()
        self.movement_data = None
