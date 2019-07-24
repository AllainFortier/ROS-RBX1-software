"""
Yes, I used to program Java and I don't put much more than one class in any modules... So be it.
"""


class ROSFollowJointTrajectoryMessageWrapper:

    def __init__(self, message):
        """
        Pass the message received from ROS through the action server.
        :param message:
        """
        self.trajectory = message.trajectory
        self.points = self.trajectory.points

    def get_times(self):
        """
        Returns a list of the times from start for points in seconds.
        :return:
        """
        ls = []
        for point in self.points:
            ls.append(point.time_from_start.secs + point.time_from_start.nsecs * 1e-9)

        return ls

    def get_joint_positions(self, id):
        ls = []
        for point in self.points:
            ls.append(point.positions[id])

    def get_joint_velocities(self, id):
        ls = []
        for point in self.points:
            ls.append(point.velocities[id])

    def get_joint_accelerations(self, id):
        ls = []
        for point in self.points:
            ls.append(point.accelerations[id])

    @property
    def joint_count(self):
        """
        Returns how many joints are in the message
        :return:
        """
        return len(self.trajectory.joint_names)

