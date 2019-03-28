from inputs import get_gamepad
import RPi.GPIO as GPIO
import Slush
import Slush.Devices.L6470Registers as LReg
import math
import time


class RBX1:

    # setup all of the axis for the SlushEngine
    b = Slush.sBoard()
    joints = [Slush.Motor(0), Slush.Motor(1), Slush.Motor(2), Slush.Motor(3), Slush.Motor(4), Slush.Motor(5)]
    pwm = None
    gripper = 7

    def setup(self):

        for joint in self.joints:
            joint.resetDev()
            joint.setMicroSteps(16)

        # some initialization stuff that needs cleanup
        self.joints[0].setMaxSpeed(150)
        self.joints[1].setMaxSpeed(150)
        self.joints[2].setMaxSpeed(150)
        self.joints[3].setMaxSpeed(150)
        self.joints[4].setMaxSpeed(150)
        self.joints[5].setMaxSpeed(150)

        # joint current limits. Still setting manually because testing (hold A, run A, acc A, dec, A)
        # self.joints[0].setCurrent(75, 85, 75, 70)

        curr = 55
        self.joints[0].setCurrent(curr, curr, curr, curr)
        self.joints[0].setOverCurrent(1500)
        self.joints[0].setLowSpeedOpt(0)
        
        # self.joints[1].setCurrent(75, 70, 80, 75)
        # self.joints[2].setCurrent(50, 50, 50, 50)
        curr_2 = 55
        self.joints[2].setCurrent(curr_2, curr_2, curr_2, curr_2)
        self.joints[2].setOverCurrent(375)
        self.joints[2].setLowSpeedOpt(1)
        # self.joints[3].setCurrent(75, 75, 75, 75)
        # self.joints[4].setCurrent(85, 85, 85, 85)
        # self.joints[5].setCurrent(65, 65, 65, 65)

        # setup the gripper
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT)
        self.pwm = GPIO.PWM(18, 100)

    def wait_for_robot(self):
        for joint in self.joints:
            while joint.isBusy():
                continue

    def print_positions(self):
        pos = []

        for joint in self.joints:
            pos.append(joint.getPosition())

        print("Positions: {}".format(pos))

    def get_currents(self, motor):
        temp = [motor.getParam(LReg.KVAL_RUN),
                motor.getParam(LReg.KVAL_ACC),
                motor.getParam(LReg.KVAL_DEC),
                motor.getParam(LReg.KVAL_HOLD)]

        return temp

    def print_status(self):
        nb = 0
        status = self.joints[nb].getStatus()
        
        print("Status: {0:016b}".format(status))
        print("ADC_OUT: {:05b}".format(self.joints[nb].getParam(LReg.ADC_OUT)))
        print("Config: {:07b}".format(self.joints[nb].getParam(LReg.STALL_TH)))
  
        if not status & LReg.STATUS_STEP_LOSS_A:
            print("Step LOSS A")
        if not status & LReg.STATUS_STEP_LOSS_B:
            print("Step LOSS B")
        if not status & LReg.STATUS_UVLO:
            print("Under voltage lockout")
        if not status & LReg.STATUS_OCD:
            print("Overcurrent")
        if not status & LReg.STATUS_TH_WRN:
            print(" ##### Thermal Warning ##### ")
            # raise Exception("THERMAL WARNING")
        if not status & LReg.STATUS_BUSY:
            print(" Busy ")
            # raise Exception("THERMAL WARNING")

    def run_robot(self):
        # start reading the inputs from the gamepad and putting them out the joints
        while 1:
            print("Time: {}".format(time.time()))
            self.print_positions()
            self.print_status()
            
            events = get_gamepad()
            for event in events:
                
                if event.code == 'BTN_MODE':
                    value = event.state
                    if value == 1:
                        for joint in self.joints:
                            joint.free()

                if event.code == 'ABS_Y':
                    value = event.state
                    self.print_status()
                    if value < -1500:
                        if not self.joints[0].isBusy():
                            self.joints[0].run(0, 35)
                    elif value > 5000:
                        if not self.joints[0].isBusy():
                            self.joints[0].run(1, 35)
                    else:
                        if not self.joints[0].isBusy():
                            self.joints[0].softStop()
                if event.code == 'ABS_X':
                    value = event.state
                    if value < -1500:
                        if not self.joints[1].isBusy():
                            self.joints[1].run(1, 20)
                    elif value > 5000:
                        if not self.joints[1].isBusy():
                            self.joints[1].run(0, 20)
                    else:
                        if not self.joints[1].isBusy():
                            self.joints[1].softStop()
                if event.code == 'ABS_RX':
                    value = event.state
                    if value < -3500:
                        if not self.joints[2].isBusy():
                            self.joints[2].run(1, 100)
                    elif value > 3500:
                        if not self.joints[2].isBusy():
                            self.joints[2].run(0, 100)
                    else:
                        if not self.joints[2].isBusy():
                            self.joints[2].softStop()
                if event.code == 'ABS_RY':
                    value = event.state
                    if value < -3500:
                        if not self.joints[3].isBusy():
                            self.joints[3].run(1, 10)
                    elif value > 3500:
                        if not self.joints[3].isBusy():
                            self.joints[3].run(0, 10)
                    else:
                        if not self.joints[3].isBusy():
                            self.joints[3].softStop()
                if event.code == 'ABS_HAT0Y':
                    value = event.state
                    if value == 1:
                        if not self.joints[4].isBusy():
                            self.joints[4].run(1, 20)
                    elif value == -1:
                        if not self.joints[4].isBusy():
                            self.joints[4].run(0, 20)
                    else:
                        if not self.joints[4].isBusy():
                            self.joints[4].softStop()
                if event.code == 'ABS_HAT0X':
                    value = event.state
                    if value == 1:
                        if not self.joints[5].isBusy():
                            self.joints[5].run(1, 20)
                    elif value == -1:
                        if not self.joints[5].isBusy():
                            self.joints[5].run(0, 20)
                    else:
                        if not self.joints[5].isBusy():
                            self.joints[5].softStop()

                # Opens and closes the gripper
                if event.code == 'BTN_TL':
                    if event.state == 1:
                        self.gripper -= 1
                        if self.gripper < 7:
                            self.gripper = 7
                        self.pwm.start(self.gripper)
                if event.code == 'BTN_TR':
                    if event.state == 1:
                        self.gripper += 1
                        if self.gripper > 17:
                            self.gripper = 17
                        self.pwm.start(self.gripper)

                # calibrate all axis if on point
                if event.code == 'BTN_START':
                    if event.state == 1:
                        for joint in self.joints:
                            joint.setAsHome()


if __name__ == '__main__':
    robot = RBX1()
    try:
        robot.setup()
        robot.run_robot()
    except KeyboardInterrupt as e:
        print("Keyboard Interrupt. Stopping sotfware.")
    finally:
        GPIO.cleanup()
