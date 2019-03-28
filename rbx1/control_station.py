import math
import queue

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import qApp

import robot_arm
from controller import stepper_motor
from controller.status import L6470StatusMask, L6480StatusMask, L6470Status, L6480Status
from threads.ReceiverBridge import ReceiverBridge
from ui.mainwindow import Ui_MainWindow


class ControlStation(QtWidgets.QMainWindow):

    def __init__(self, robot_arm: robot_arm.RobotArm, trajectory_queue: queue.Queue, gripper_queue: queue.Queue):
        super(ControlStation, self).__init__()

        self.robot_arm = robot_arm
        self.trajectory_queue = trajectory_queue
        self.gripper_queue = gripper_queue
        self.selected_motor = self.robot_arm.motors[0]

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Build the trajectory bridge and queue
        self.thread_arm = QtCore.QThread()
        self.bridge_arm = ReceiverBridge(trajectory_queue, topic="arm")
        self.bridge_arm.moveToThread(self.thread_arm)

        # Build the gripper command bridge and queue
        self.thread_gripper = QtCore.QThread()
        self.bridge_gripper = ReceiverBridge(gripper_queue, topic="gripper")
        self.bridge_gripper.moveToThread(self.thread_gripper)

        # Assigns signals to just created thread
        self.bridge_arm.signal_data_received.connect(self.trajectory_data_received)
        self.bridge_gripper.signal_data_received.connect(self.gripper_command_received)
        self.thread_arm.started.connect(self.bridge_arm.start)
        self.thread_gripper.started.connect(self.bridge_gripper.start)
        qApp.aboutToQuit.connect(self.thread_arm.quit)
        qApp.aboutToQuit.connect(self.thread_gripper.quit)

        self.thread_arm.start()
        self.thread_gripper.start()

        self.timer_rate = 400
        QtCore.QTimer.singleShot(self.timer_rate, self.refresh_data)

        self.populate_motors()

        self.connect_all()
        self.set_validators()
        self.set_slider_min_max()

        self.L6470 = L6470Status()
        self.L6480 = L6480Status()

        # Creates the status monitor and generates the binding between Q objects and registers
        self.L6470.set_view(L6470StatusMask.HIZ, self.ui.pushButtonHiZ)
        self.L6470.set_view(L6470StatusMask.BUSY, self.ui.pushButtonBusy)
        self.L6470.set_view(L6470StatusMask.SW_F, self.ui.pushButtonSW_F)
        self.L6470.set_view(L6470StatusMask.SW_EVN, self.ui.pushButtonSW_EVN)
        self.L6470.set_view(L6470StatusMask.TH_WRN, self.ui.pushButtonTH_WRN)
        self.L6470.set_view(L6470StatusMask.TH_SD, self.ui.pushButtonTH_SD)
        self.L6470.set_view(L6470StatusMask.OCD, self.ui.pushButtonOCD)
        self.L6470.set_view(L6470StatusMask.STEP_LOSS_A, self.ui.pushButtonSTEP_LOSS_A)
        self.L6470.set_view(L6470StatusMask.STEP_LOSS_B, self.ui.pushButtonSTEP_LOSS_B)
        self.L6470.set_view(L6470StatusMask.SCK_MOD, self.ui.pushButtonSCK_MOD)
        # self.L6470.set_view(L6470StatusMask.MOT_STATUS, self.ui.pushButtonMOT_STATUS)

        self.L6480.set_view(L6480StatusMask.HIZ, self.ui.pushButtonHiZ)
        self.L6480.set_view(L6480StatusMask.BUSY, self.ui.pushButtonBusy)
        self.L6480.set_view(L6480StatusMask.SW_F, self.ui.pushButtonSW_F)
        self.L6480.set_view(L6480StatusMask.SW_EVN, self.ui.pushButtonSW_EVN)
        self.L6480.set_view(L6480StatusMask.WRONG_CMD, self.ui.pushButtonWRONG_CMD)
        # self.L6480.set_view(L6480StatusMask.TH_STATUS, self.ui.pushButtonTH_SD)
        self.L6480.set_view(L6480StatusMask.OCD, self.ui.pushButtonOCD)
        self.L6480.set_view(L6480StatusMask.STEP_LOSS_A, self.ui.pushButtonSTEP_LOSS_A)
        self.L6480.set_view(L6480StatusMask.STEP_LOSS_B, self.ui.pushButtonSTEP_LOSS_B)
        self.L6480.set_view(L6480StatusMask.SCK_MOD, self.ui.pushButtonSCK_MOD)
        # self.L6480.set_view(L6480StatusMask.MOT_STATUS, self.ui.pushButtonMOT_STATUS)

    def connect_all(self):
        self.ui.listMotors.itemClicked.connect(self.select_motor)

        # Buttons connections
        self.ui.pushFree.clicked.connect(self.btn_free)
        self.ui.pushSoftStop.clicked.connect(self.btn_soft_stop)
        self.ui.pushHardStop.clicked.connect(self.btn_hard_stop)
        self.ui.pushEasyRun.clicked.connect(self.btn_easy_run)
        self.ui.pushButtonHIZAll.clicked.connect(self.hiz_all)
        self.ui.pushFreeAll.clicked.connect(self.free_all)
        self.ui.pushButtonMove.clicked.connect(self.move)
        self.ui.pushButtonGoHome.clicked.connect(self.go_home)

        self.ui.pushButtonOpenGripper.clicked.connect(self.open_gripper)
        self.ui.pushButtonCloseGripper.clicked.connect(self.close_gripper)

    def set_validators(self):
        pass

    def open_gripper(self):
        self.robot_arm.gripper.open()

    def close_gripper(self):
        self.robot_arm.gripper.close()

    def btn_free(self):
        self.selected_motor.free()

    def btn_soft_stop(self):
        self.selected_motor.softStop()

    def btn_hard_stop(self):
        self.selected_motor.hardStop()

    def btn_easy_run(self):
        self.selected_motor.run(0, 30)

    def refresh_data(self):
        try:
            self.refresh_motor(self.selected_motor)
            if self.selected_motor.index <= 2:
                self.L6480.update_status(self.selected_motor.getStatus())
            else:
                self.L6470.update_status(self.selected_motor.getStatus())

        finally:
            QtCore.QTimer.singleShot(self.timer_rate, self.refresh_data)

    def hiz_all(self):
        for motor in self.robot_arm.motors:
            motor.softStop()

    def free_all(self):
        for motor in self.robot_arm.motors:
            motor.free()

    def trajectory_data_received(self):
        print("Trajectory data received")
        data = self.trajectory_queue.get()
        self.robot_arm.execute_movement(data.trajectory)

    def gripper_command_received(self):
        print("Gripper command data received")
        data = self.gripper_queue.get()
        # self.robot_arm.gripper.execute_movement(data.trajectory)

    def set_slider_min_max(self):
        sliders = [
            self.ui.horizontalSliderJoint_1,
            self.ui.horizontalSliderJoint_2,
            self.ui.horizontalSliderJoint_3,
            self.ui.horizontalSliderJoint_4,
            self.ui.horizontalSliderJoint_5,
            self.ui.horizontalSliderJoint_6
        ]

        for slider, motor_config in zip(sliders, self.robot_arm.config['Motors']):
            min_bound = float(motor_config['lower_limit'])
            slider.setMinimum(math.degrees(min_bound))

            max_bound = float(motor_config['upper_limit'])
            slider.setMaximum(math.degrees(max_bound))

    def move(self):
        motors = self.robot_arm.motors

        sliders = [
            self.ui.horizontalSliderJoint_1,
            self.ui.horizontalSliderJoint_2,
            self.ui.horizontalSliderJoint_3,
            self.ui.horizontalSliderJoint_4,
            self.ui.horizontalSliderJoint_5,
            self.ui.horizontalSliderJoint_6
        ]

        # TODO move method that accepts an angle and rad as movement command
        motors[0].goTo(motors[0].convert_rad_to_microsteps(math.radians(sliders[0].value())))
        motors[1].goTo(motors[1].convert_rad_to_microsteps(math.radians(sliders[1].value())))
        motors[2].goTo(motors[2].convert_rad_to_microsteps(math.radians(sliders[2].value())))
        motors[3].goTo(motors[3].convert_rad_to_microsteps(math.radians(sliders[3].value())))
        motors[4].goTo(motors[4].convert_rad_to_microsteps(math.radians(sliders[4].value())))
        motors[5].goTo(motors[5].convert_rad_to_microsteps(math.radians(sliders[5].value())))

    def refresh_motor(self, motor: stepper_motor.MotorController):
        self.ui.plainTextEditPosition.setPlainText(str(motor.getPosition()))
        self.ui.plainTextEditSpeed.setPlainText(str(motor.getSpeed()))
        self.ui.plainTextEditStatus.setPlainText("{:016b}".format(motor.getStatus()))
        self.ui.textEditTarget.setText(str(motor.target_position))

    def select_motor(self, item: QtWidgets.QListWidgetItem):
        for motor in self.robot_arm.motors:
            if item.text() == motor.name:
                self.selected_motor = motor
                print("Selected: {}".format(motor.name))

    def populate_motors(self):
        for motor in self.robot_arm.motors:
            self.ui.listMotors.addItem(motor.name)

    def hiz_motor(self):
        pass

    def reset_status(self):
        self.selected_motor.getStatus()

    def go_home(self):
        for motor in self.robot_arm.motors:
            motor.goHome()

