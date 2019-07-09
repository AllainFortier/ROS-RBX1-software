from Slush.Devices.L6470Registers import *

from controller.stepper_driver import MotorController
import RPi.GPIO as GPIO


class ConfigLoader:

    def __init__(self):
        GPIO.setmode(GPIO.BCM)

    def create_motors(self, config):
        """
        Will initialise a list of motors and set their properties as defined in the configuration files.
        :param config: The configuration file
        :return: An array of the configured motors
        """
        motors = []

        for motor_config in config['Motors']:
            motor = self._create_motor(motor_config)

            motor.resetDev()
            self._apply_current(motor, motor_config)
            self._apply_mechanical_factors(motor, motor_config)
            
            motors.append(motor)

        return motors

    def _create_motor(self, config):
        print("Created motor [{}] at board id {}".format(config['name'], config['index']))
        return MotorController(config['index'], config['name'])

    def _apply_mechanical_factors(self, motor, config):
        """
        Applies mechanical factors from config file such as:
        Gear ratio, steps per revolutions and microsteps mode
        :param motor: The motor to configure
        :param config: The configuration file to use
        :return: None
        """
        motor.setMaxSpeed(config['max_speed'])
        motor.gear_ratio = config['gear_ratio']
        motor.steps_per_revolution = config['steps_per_revolution']

        motor.micro_steps = config['microstepping']
        motor.setMicroSteps(motor.micro_steps)

        motor.setLowSpeedOpt(1)

        motor.inverted = config.get('inverted', False)

    def _apply_current(self, motor, config):
        """
        Applies electrical configuration to the motor
        :param motor: The motor to apply to
        :param config: The configuration file to use
        :return: None
        """
        motor.setCurrent(
            config['hold_current'],
            config['run_current'],
            config['acc_current'],
            config['dec_current']
        )

        motor.setThresholdSpeed(50)  # TODO Confirm values
        print("ST_SLP {}".format(motor.getParam(ST_SLP)))

        motor.setOverCurrent(config['overcurrent'])

    def create_gripper(self, config):
        GPIO.setup(config['gripper_GPIO'], GPIO.OUT)
        return GPIO.PWM(config['gripper_GPIO'], config['gripper_PWM'])

    def cleanup(self):
        GPIO.cleanup()
