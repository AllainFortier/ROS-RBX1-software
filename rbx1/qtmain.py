#!/usr/bin/env python

from PyQt5.QtWidgets import QApplication
from ros.remote_rviz_command import ManualControl


def main():
    try:
        print("============ Initialising Robot Arm software!")
        app = QApplication([])

        controller = ManualControl()
        controller.show()

        app.exec_()

    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Exiting properly.")
    finally:
        app.quit()


if __name__ == '__main__':
    main()
