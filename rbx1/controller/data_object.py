"""
Yes, I used to program Java and I don't put much more than one class in any modules... So be it.
"""


class MovementData:
    """
    This data object has the purpose of storing movement data. The arm receives the ROS data, stores in in this data
    object then sends it to the motors to be executed. This is data only and no processing should be done at this level.
    """

    def __init__(self, position, velocity, acceleration, time_from_start):
        """
        Standard creation of the movement data object.
        :param position: radians
        :param velocity: radians/seconds
        :param acceleration: radians/seconds**2
        :param time_from_start: seconds
        """
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self._time_from_start = time_from_start

    @property
    def time_from_start(self):
        """
        The time from start from the movement object. Reference point is 0 seconds.
        :return: time from start in seconds
        """
        return self._time_from_start


