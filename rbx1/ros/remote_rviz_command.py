import math
import sys

import moveit_commander
import moveit_msgs
import rospy
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QCoreApplication
from PyQt5.QtWidgets import qApp
from geometry_msgs.msg import Quaternion, geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from threads.joint_state_publisher import JointStatePublisher
from ui.manualcommands import Ui_MainWindow


class ManualControl(QtWidgets.QMainWindow):

    def __init__(self):
        super(ManualControl, self).__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('rbx_1_node', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        self.group.set_goal_position_tolerance(0.05)
        self.group.set_goal_orientation_tolerance(0.05)
        self.group.set_planning_time(10)
        self.group.allow_replanning(False)
        self.group.allow_looking(True)

        # Prepares the gripper planning group
        self.gripper_name = "gripper"
        self.group_gripper = moveit_commander.MoveGroupCommander(self.gripper_name)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

        self.connect_all()
        self.set_validators()

    def connect_all(self):
        # Connect manual coordinate push buttons
        self.ui.pushButtonSetPosition.clicked.connect(self.set_position_target)
        self.ui.pushButtonShiftPosition.clicked.connect(self.shift_position_target)
        self.ui.pushButtonSetOrientation.clicked.connect(self.set_orientation_target)
        self.ui.pushButtonShiftOrientation.clicked.connect(self.shift_orientation_target)
        self.ui.pushButtonSetPoseTarget.clicked.connect(self.set_pose_target)
        self.ui.pushButtonRandomTarget.clicked.connect(self.set_random_target)
        self.ui.pushButtonRefreshStatus.clicked.connect(self.refresh_information)
        self.ui.pushButtonCloseGripper.clicked.connect(self.close_gripper)
        self.ui.pushButtonOpenGripper.clicked.connect(self.open_gripper)

        # Shift buttons
        self.ui.pushButton_X_Minus.clicked.connect(self.shift_x_minus)
        self.ui.pushButton_X_Plus.clicked.connect(self.shift_x_plus)
        self.ui.pushButton_Y_Minus.clicked.connect(self.shift_y_minus)
        self.ui.pushButton_Y_Plus.clicked.connect(self.shift_y_plus)
        self.ui.pushButton_Z_Minus.clicked.connect(self.shift_z_minus)
        self.ui.pushButton_Z_Plus.clicked.connect(self.shift_z_plus)
        self.ui.pushButtonRandomTarget.clicked.connect(self.set_random_target)
        self.ui.pushButtonSetJointTarget.clicked.connect(self.set_joint_target)

        # Connect menu bar actions
        self.ui.actionExit.triggered.connect(self.exit)

    def set_validators(self):
        sliders = [
            self.ui.horizontalSliderJoint_6,
            self.ui.horizontalSliderJoint_5,
            self.ui.horizontalSliderJoint_4,
            self.ui.horizontalSliderJoint_3,
            self.ui.horizontalSliderJoint_2,
            self.ui.horizontalSliderJoint_1
        ]

        joints = ['Joint_6', 'Joint_5', 'Joint_4', 'Joint_3', 'Joint_2', 'Joint_1']

        for slider, joint in zip(sliders, joints):
            min_bound = self.robot.get_joint(joint).min_bound()
            slider.setMinimum(math.degrees(min_bound))

            max_bound = self.robot.get_joint(joint).max_bound()
            slider.setMaximum(math.degrees(max_bound))

    def set_joint_target(self):
        joint_goal = self.group.get_current_joint_values()

        sliders = [
            self.ui.horizontalSliderJoint_6,
            self.ui.horizontalSliderJoint_5,
            self.ui.horizontalSliderJoint_4,
            self.ui.horizontalSliderJoint_3,
            self.ui.horizontalSliderJoint_2,
            self.ui.horizontalSliderJoint_1
        ]

        joint_goal[5] = math.radians(sliders[0].value())
        joint_goal[4] = math.radians(sliders[1].value())
        joint_goal[3] = math.radians(sliders[2].value())
        joint_goal[2] = math.radians(sliders[3].value())
        joint_goal[1] = math.radians(sliders[4].value())
        joint_goal[0] = math.radians(sliders[5].value())

        self.group.go(joint_goal)
        self.group.stop()

    def open_gripper(self):
        pass
        """
        joint_goal = self.group_gripper.get_current_joint_values()
        print(joint_goal)
        joint_goal[0] = 0

        self.group_gripper.go(joint_goal)
        self.group_gripper.stop()
        """

    def close_gripper(self):
        pass
        """
        joint_goal = self.group_gripper.get_current_joint_values()
        print(joint_goal)
        joint_goal[0] = -1.57

        self.group_gripper.go(joint_goal)
        self.group_gripper.stop()
        """

    def exit(self):
        QCoreApplication.quit()

    def set_position_target(self):
        # Sets the targets for the arm group
        x = float(self.ui.plainTextEdit_X.toPlainText())
        y = float(self.ui.plainTextEdit_Y.toPlainText())
        z = float(self.ui.plainTextEdit_Z.toPlainText())

        self.group.set_position_target([x, y, z])
        print("Set Position Target ({:.4f}, {:.4f}, {:.4f})".format(x, y, z))

        self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()

    def shift_position_target(self):
        pose = self.group.get_current_pose().pose

        # Sets the targets for the arm group
        x = float(self.ui.plainTextEdit_X.toPlainText())
        y = float(self.ui.plainTextEdit_Y.toPlainText())
        z = float(self.ui.plainTextEdit_Z.toPlainText())

        pose.position.x += x
        pose.position.y += y
        pose.position.z += z

        print("Shift Position Target ({:.4f}, {:.4f}, {:.4f})".format(x, y, z))
        self.group.set_pose_target(pose)

        self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()

    def set_pose_target(self):
        pose_goal = geometry_msgs.msg.Pose()

        x = float(self.ui.plainTextEdit_X.toPlainText())
        y = float(self.ui.plainTextEdit_Y.toPlainText())
        z = float(self.ui.plainTextEdit_Z.toPlainText())

        roll = float(self.ui.plainTextEdit_Roll.toPlainText())
        pitch = float(self.ui.plainTextEdit_Pitch.toPlainText())
        yaw = float(self.ui.plainTextEdit_Yaw.toPlainText())

        quat = quaternion_from_euler(roll, pitch, yaw)
        pose_goal.orientation = Quaternion(*quat)

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        # Sets the targets for the arm group
        self.group.set_pose_target(pose_goal)

        self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()

    def set_orientation_target(self):
        roll = float(self.ui.plainTextEdit_Roll.toPlainText())
        pitch = float(self.ui.plainTextEdit_Pitch.toPlainText())
        yaw = float(self.ui.plainTextEdit_Yaw.toPlainText())

        quat = quaternion_from_euler(roll, pitch, yaw)

        # Sets the targets for the arm group
        self.group.set_orientation_target(quat)

        self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()

    def shift_orientation_target(self):
        pose = self.group.get_current_pose().pose

        # Sets the targets for the arm group
        roll = float(self.ui.plainTextEdit_Roll.toPlainText())
        pitch = float(self.ui.plainTextEdit_Pitch.toPlainText())
        yaw = float(self.ui.plainTextEdit_Yaw.toPlainText())

        temp_quat = self.group.get_current_pose().pose.orientation
        euler = euler_from_quaternion([temp_quat.x, temp_quat.y, temp_quat.z, temp_quat.w])
        euler_shifted = (euler[0] + roll, euler[1] + pitch, euler[2] + yaw)

        quat = quaternion_from_euler(euler_shifted[0], euler_shifted[1], euler_shifted[0])
        pose.orientation = Quaternion(*quat)

        self.group.set_pose_target(pose)
        print("Shift Orientation Initial ({:.4f}, {:.4f}, {:.4f})".format(euler[0], euler[1], euler[2]))
        print("Shift Orientation Target ({:.4f}, {:.4f}, {:.4f})".format(roll, pitch, yaw))
        print("Shift Orientation Result ({:.4f}, {:.4f}, {:.4f})".format(euler_shifted[0], euler_shifted[1],
                                                                         euler_shifted[2]))

        self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()

    def set_random_target(self):
        # self.group.set_random_target()
        pose = self.group.get_random_pose()
        self.group.set_pose_target(pose)

        self.group.go(wait=False)
        # self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        # self.group.stop()

        self.group.clear_pose_targets()

    def refresh_information(self):
        print(self.group.get_current_pose().pose)
        temp_quat = self.group.get_current_pose().pose.orientation
        euler = euler_from_quaternion([temp_quat.x, temp_quat.y, temp_quat.z, temp_quat.w])
        print("euler orientation:\n  roll:  {:2f}\n  pitch:  {:2f}\n  yaw:  {:2f}".format(
            math.degrees(euler[0]),
            math.degrees(euler[1]),
            math.degrees(euler[2])))

    def shift_x_plus(self):
        self.group.shift_pose_target(0, 0.05)
        self.group.go(wait=True)
        self.group.stop()
        self.group.set_start_state_to_current_state()
        self.group.clear_pose_targets()

    def shift_x_minus(self):
        self.group.shift_pose_target(0, -0.05)
        self.group.go(wait=True)
        self.group.stop()
        self.group.set_start_state_to_current_state()
        self.group.clear_pose_targets()

    def shift_y_plus(self):
        self.group.shift_pose_target(1, 0.05)
        self.group.go(wait=True)
        self.group.stop()
        self.group.set_start_state_to_current_state()
        self.group.clear_pose_targets()

    def shift_y_minus(self):
        self.group.shift_pose_target(1, -0.05)
        self.group.go(wait=True)
        self.group.stop()
        self.group.set_start_state_to_current_state()
        self.group.clear_pose_targets()

    def shift_z_plus(self):
        self.group.shift_pose_target(2, 0.05)
        self.group.go(wait=True)
        self.group.stop()
        self.group.set_start_state_to_current_state()
        self.group.clear_pose_targets()

    def shift_z_minus(self):
        self.group.shift_pose_target(2, -0.05)
        self.group.go(wait=True)
        self.group.stop()
        self.group.set_start_state_to_current_state()
        self.group.clear_pose_targets()

    def print_status(self):
        pass
        """
            print("=== printing robot state")
            print(robot.get_current_state())
    
            print("=== printing end effector pose")
            print(group.get_current_pose("Link_5").pose)
            print(group.get_end_effector_link())
    
            # group.set_start_state_to_current_state()
    
            orient = group.get_current_pose("Link_5").pose.orientation
            orient_list = [orient.x, orient.y, orient.z, orient.w]
            quat = euler_from_quaternion(orient_list)
            print(quat)
    
            text = raw_input("Enter destination (x, y, z): ")
            orientation = raw_input("Enter orientation (roll, pitch, yaw): ")
            angles = text.split(' ')
            orientations = orientation.split(' ')
            # goto(angles)
            goto_pose(angles, orientations)
            """

    def goto_pose(pose, orientation):
        pass
        """
        pose_goal = geometry_msgs.msg.Pose()
    
        ee_link = "Link_5"
    
        quat = quaternion_from_euler(float(orientation[0]), float(orientation[1]), float(orientation[2]))
        # print(quat)
        # pose_goal.orientation = Quaternion(*quat)
    
        pose_goal.position.x = float(pose[0])
        pose_goal.position.y = float(pose[1])
        pose_goal.position.z = float(pose[2])
    
        # random_pose = group.get_random_pose(ee_link)
        # print(random_pose)
    
        # Sets the targets for the arm group
        # group.set_pose_target(pose_goal, ee_link)
        # group.set_pose_target(random_pose, ee_link)
        # group.set_orientation_target(quat, ee_link)
        # group.set_position_target([float(pose[0]), float(pose[1]), float(pose[2])], ee_link)
        group.shift_pose_target(2, -0.05)
        group.shift_pose_target(0, 0.05)
        group.shift_pose_target(4, 0.74)
        group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
    
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()
        """


class WaypointGenerator:

    def __init__(self):
        pass

    def load_from_file(self, file):
        pass
