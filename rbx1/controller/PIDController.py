#!/usr/bin/python


import time


class PIDController:

    def __init__(self, Kp=0.2, Kd=0, Ki=0):

        self._Kp = Kp
        self._Kd = Kd
        self._Ki = Ki

        self._sample_time = 0
        self._current_time = time.time()
        self.last_time = self._current_time

        self._feedback = 0
        self._output = 0

        self.target_value = 0.0

        self._PTerm = 0.0
        self._ITerm = 0.0
        self._DTerm = 0.0
        self._last_error = 0.0

        # Windup Guard
        self._int_error = 0.0
        self._windup_guard = 20.0

        self.reset()

    def reset(self):
        """Clears PID computations and coefficients"""
        self.target_value = 0.0

        self._PTerm = 0.0
        self._ITerm = 0.0
        self._DTerm = 0.0
        self._last_error = 0.0

        # Windup Guard
        self._int_error = 0.0
        self._windup_guard = 20.0

        self._output = 0.0

    def update(self, feedback):
        self._feedback = feedback  # Some data for debug purposes
        error = self.target_value - feedback

        self._current_time = time.time()
        delta_time = self._current_time - self.last_time
        delta_error = error - self._last_error

        if delta_time >= self._sample_time:
            self._PTerm = self.Kp * error
            self._ITerm += error * delta_time

            if self._ITerm < -self._windup_guard:
                self._ITerm = -self._windup_guard
            elif self._ITerm > self._windup_guard:
                self._ITerm = self._windup_guard

            self._DTerm = 0.0
            if delta_time > 0:
                self._DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self._current_time
            self._last_error = error

            self._output = self._PTerm + (self.Ki * self._ITerm) + (self.Kd * self._DTerm)

    def __str__(self):
        return "PID- Target:{} |Feedback:{} |Err:{} |Out:{}".format(self.target_value, self._feedback, self._last_error, self.output)

    @property
    def Kp(self):
        return self._Kp

    @Kp.setter
    def Kp(self, val):
        self._Kp = val

    @property
    def Kd(self):
        return self._Kd

    @Kd.setter
    def Kd(self, val):
        self._Kd = val

    @property
    def Ki(self):
        return self._Ki

    @Ki.setter
    def Ki(self, val):
        self._Ki = val

    @property
    def windup(self):
        return self._windup_guard

    @windup.setter
    def windup(self, val):
        self._windup_guard = val

    @property
    def sample_time(self):
        return self._sample_time

    @sample_time.setter
    def sample_time(self, val):
        self._sample_time = val

    @property
    def output(self):
        return self._output



