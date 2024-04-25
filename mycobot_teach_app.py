import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel, QListWidget, \
    QMessageBox, QSpinBox
from PyQt5 import QtCore
from pymycobot import MyCobotSocket
import time


class MyCobotApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.mc = None
        self.stop_record_flag = False
        self.commands = []  # This list will hold the saved commands
        self.initUI()

        self.timer = QtCore.QTimer(self)  # Initialize the timer
        self.timer.setInterval(500)  # Set the interval to 1000 milliseconds (1 second)
        # self.timer.timeout.connect(self.update_angles)  # Connect the timeout signal to the update_angles method

    def initUI(self):
        self.setWindowTitle('MyCobot Controller')

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        layout = QVBoxLayout()

        self.connect_button = QPushButton('Connect to Robot')
        self.connect_button.clicked.connect(self.connect_robot)
        layout.addWidget(self.connect_button)

        self.home_button = QPushButton('Go Home')
        self.home_button.clicked.connect(self.go_home)
        layout.addWidget(self.home_button)

        self.release_button = QPushButton('Release Servos')
        self.release_button.clicked.connect(self.release_servos)
        layout.addWidget(self.release_button)

        self.play_commands_button = QPushButton('Play Commands')
        self.play_commands_button.clicked.connect(self.play_commands)
        layout.addWidget(self.play_commands_button)

        self.delete_commands_button = QPushButton('Delete ALL Commands')
        self.delete_commands_button.clicked.connect(self.delete_commands)
        layout.addWidget(self.delete_commands_button)

        self.delete_last_command_button = QPushButton('Delete Last Command')
        self.delete_last_command_button.clicked.connect(self.delete_last_command)
        layout.addWidget(self.delete_last_command_button)

        self.commands_list = QListWidget()
        layout.addWidget(self.commands_list)

        # self.angle_label = QLabel('Current Angles: Unknown')
        # layout.addWidget(self.angle_label)

        self.save_current_position_button = QPushButton('Save Current Position')
        self.save_current_position_button.clicked.connect(self.save_current_position)
        layout.addWidget(self.save_current_position_button)

        self.save_close_gripper_button = QPushButton('Save Close Gripper Command')
        self.save_close_gripper_button.clicked.connect(self.save_close_gripper_command)
        layout.addWidget(self.save_close_gripper_button)

        self.save_open_gripper_button = QPushButton('Save Open Gripper Command')
        self.save_open_gripper_button.clicked.connect(self.save_open_gripper_command)
        layout.addWidget(self.save_open_gripper_button)

        self.record_trajectory_button = QPushButton('Record Trajectory')
        self.record_trajectory_button.clicked.connect(self.record_trajectory)
        layout.addWidget(self.record_trajectory_button)

        # Create a QLabel for the QSpinBox
        self.spin_box_label = QLabel('Speed:')
        layout.addWidget(self.spin_box_label)

        self.speed_spin_box = QSpinBox()
        self.speed_spin_box.setMinimum(0)
        self.speed_spin_box.setMaximum(100)
        self.speed_spin_box.setValue(25)  # Default value
        layout.addWidget(self.speed_spin_box)

        # Exit button
        self.exit_button = QPushButton('Exit')
        self.exit_button.clicked.connect(self.close_app)  # Connect the button to close_app method
        layout.addWidget(self.exit_button)

        self.central_widget.setLayout(layout)

    def connect_robot(self):
        # Connect to the MyCobot
        self.mc = MyCobotSocket('141.44.152.231', 9000)
        print('Connected to robot.')
        self.timer.start()  # Start the timer

    def disconnect_robot(self):
        if self.mc:
            self.mc.close()
            self.mc = None
            print('Disconnected from robot.')

    def go_home(self):
        if self.mc:

            home_angles = [2.72, 12.39, -96.59, -0.26, -3.07, 142.64]
            self.mc.send_angles(home_angles, 50)
            start = time.time()
            self.mc.set_color(255, 165, 0)
            while True:
                if self.mc.is_in_position(home_angles, 0):
                    break
                if time.time() - start > 15:
                    print('Timeout')
                    self.mc.set_color(255, 0, 0)
                    return  # something went wrong
            # open gripper
            self.mc.set_gripper_state(0, 50)
            time.sleep(2)
            self.mc.set_color(0, 255, 0)
            print('Moved to home position.')

    def release_servos(self):
        # wait for 1 second
        time.sleep(1)
        # set atom to blue color
        self.mc.set_color(0, 0, 255)
        if self.mc:
            self.mc.release_all_servos()
            print('Released all servos.')

    def play_commands(self):

        speed = self.speed_spin_box.value()
        # Play commands from the list
        print('Playing commands...')
        if self.mc:
            # set atom to red color
            self.mc.set_color(255, 165, 0)
            for command in self.commands:
                match command[0]:
                    case 'angle':
                        print(f'Going to position: {command[1]}')
                        self.mc.send_angles(command[1], speed)
                        start = time.time()
                        while True:
                            if self.mc.is_in_position(command[1], 0):
                                break
                            if time.time() - start > 15:
                                print('Timeout')
                                self.mc.set_color(255, 0, 0)
                                return  # something went wrong
                    case 'gripper':
                        if command[1] == 'close':
                            print('Closing gripper')
                            self.mc.set_gripper_state(1, speed)

                        elif command[1] == 'open':
                            print('Opening gripper')
                            self.mc.set_gripper_state(0, speed)

                        time.sleep(2)
                    case _:
                        print('Unknown command')
        print('Commands played.')
        # set atom to green color
        self.mc.set_color(0, 255, 0)

    def delete_commands(self):
        self.commands = []
        self.commands_list.clear()
        print('Commands deleted.')

    def delete_last_command(self):
        if self.commands:
            self.commands.pop()
            self.commands_list.takeItem(self.commands_list.count() - 1)
            print('Last command deleted.')
        else:
            print('No commands to delete.')

    def save_current_position(self):
        if self.mc:
            angles = self.mc.get_angles()
            if angles is not None:
                self.commands.append(('angle', angles))
                self.commands_list.addItem(f'Position: {angles}')
                print('Command saved.')
            else:
                print('Angles was None. Could not save command.')

    def record_trajectory(self):
        # set recording button to stop
        self.record_trajectory_button.setText('Stop Recording')
        self.record_trajectory_button.clicked.disconnect()
        self.record_trajectory_button.clicked.connect(self.stop_recording)
        if self.mc:
            while True:
                if self.stop_record_flag:
                    self.stop_record_flag = False
                    return
                angles = self.mc.get_angles()
                if angles is not None:
                    self.commands.append(('angle', angles))
                    self.commands_list.addItem(f'Position: {angles}')
                    print('Command saved.')
                time.sleep(0.1)
                # update ui
                QApplication.processEvents()

    def stop_recording(self):
        self.stop_record_flag = True
        # set recording button to start
        self.record_trajectory_button.setText('Record Trajectory')
        self.record_trajectory_button.clicked.disconnect()
        self.record_trajectory_button.clicked.connect(self.record_trajectory)

    def save_close_gripper_command(self):
        if self.mc:
            self.commands.append(('gripper', 'close'))
            self.commands_list.addItem(f'Gripper: close')
            print('Command saved.')

    def save_open_gripper_command(self):
        if self.mc:
            self.commands.append(('gripper', 'open'))
            self.commands_list.addItem(f'Gripper: open')
            print('Command saved.')

    def update_angles(self):
        # Fetch and update angles in the GUI periodically
        if self.mc:
            current_angles = self.mc.get_angles()
            self.angle_label.setText(f'Current Angles: {current_angles}')
            print(f'Updated Angles: {current_angles}')  # Optional, for debugging+

    def close_app(self):
        # Close the application
        self.disconnect_robot()
        self.timer.stop()
        self.close()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MyCobotApp()
    ex.show()
    sys.exit(app.exec_())


def run_app():
    app = QApplication(sys.argv)
    ex = MyCobotApp()
    ex.show()
    sys.exit(app.exec_())
