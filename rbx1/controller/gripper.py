import time


class Gripper:

    def __init__(self, pwn):
        self.pwn = pwn

        self.pwn.start(7)
        time.sleep(2)
        self.pwn.ChangeDutyCycle(0)

        self.min = 5
        self.max = 12

    def open(self):
        self.pwn.ChangeDutyCycle(self.min)
        time.sleep(1)
        self.pwn.ChangeDutyCycle(0)

    def close(self):
        self.pwn.ChangeDutyCycle(self.max)
        time.sleep(1)
        self.pwn.ChangeDutyCycle(0)

    def set_position(self, position):
        self.pwn.start(position)

    def get_position(self):
        pass
