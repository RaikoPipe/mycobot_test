import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel, QListWidget, \
    QMessageBox, QSpinBox
from PyQt5 import QtCore
from pymycobot import MyCobotSocket, MyCobot
import time
import threading
import cv2_rec

t = [199.6, -30, 99.1, 178.14, 7.62, 161.28]
s = [174.5, 21.3]
d = [15.1, 8.7]


class MyCobotApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.current_thread = None
        self.mc = None
        self.home_angles = [2.54, 55.72, -97.73, -44.03, -1.66, 140.0]
        self.home_coords = [52.9, -62.6, 217.8, 178.98, 1.95, 132.06]
        self.stop_record_flag = False
        self.commands = []  # This list will hold the saved commands
        self.init_ui()

        self.timer = QtCore.QTimer(self)  # Initialize the timer
        self.timer.setInterval(2000)  # Set the interval to 1000 milliseconds (1 second)
        self.timer.timeout.connect(self.update_angles)  # Connect the timeout signal to the update_angles method

    def init_ui(self):
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

        self.go_to_cube_button = QPushButton('Go to Cube')
        self.go_to_cube_button.clicked.connect(self.go_to_cube)
        layout.addWidget(self.go_to_cube_button)

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

        self.angle_label = QLabel('Current Angles: Unknown')
        layout.addWidget(self.angle_label)
        self.coord_label = QLabel('Current Coords: Unknown')
        layout.addWidget(self.coord_label)

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
        self.spin_box_label_speed = QLabel('Speed:')
        layout.addWidget(self.spin_box_label_speed)

        self.speed_spin_box = QSpinBox()
        self.speed_spin_box.setMinimum(0)
        self.speed_spin_box.setMaximum(100)
        self.speed_spin_box.setValue(25)  # Default value
        layout.addWidget(self.speed_spin_box)

        # Create a QLabel for the QSpinBox
        self.spin_box_label_x_offset = QLabel('X Offset:')
        layout.addWidget(self.spin_box_label_x_offset)

        self.x_offset_spin_box = QSpinBox()
        self.x_offset_spin_box.setMinimum(-300)
        self.x_offset_spin_box.setMaximum(300)
        self.x_offset_spin_box.setValue(46)
        layout.addWidget(self.x_offset_spin_box)

        # Create a QLabel for the QSpinBox
        self.spin_box_label_y_offset = QLabel('Y Offset:')
        layout.addWidget(self.spin_box_label_y_offset)

        self.y_offset_spin_box = QSpinBox()
        self.y_offset_spin_box.setMinimum(-300)
        self.y_offset_spin_box.setMaximum(300)
        self.y_offset_spin_box.setValue(-12)
        layout.addWidget(self.y_offset_spin_box)

        # Create a QLabel for the QSpinBox
        self.spin_box_label_rot_offset = QLabel('Rot Offset:')
        layout.addWidget(self.spin_box_label_rot_offset)

        self.rot_offset_spin_box = QSpinBox()
        self.rot_offset_spin_box.setMinimum(-45)
        self.rot_offset_spin_box.setMaximum(45)
        self.rot_offset_spin_box.setValue(0)
        layout.addWidget(self.rot_offset_spin_box)

        # Exit button
        self.exit_button = QPushButton('Exit')
        self.exit_button.clicked.connect(self.close_app)  # Connect the button to close_app method
        layout.addWidget(self.exit_button)

        self.central_widget.setLayout(layout)

    def connect_robot(self):
        # Connect to the MyCobot
        #self.mc = MyCobotSocket('141.44.152.231', 9000)
        self.mc = MyCobot('/dev/ttyTHS1', 1000000)
        print('Connected to robot.')
        self.timer.start()  # Start the timer

    def disconnect_robot(self):
        if self.mc:
            self.mc.close()
            self.mc = None
            print('Disconnected from robot.')

    def go_home(self):
        def move_home():
            if self.mc:
                arrived = self.move_cobot_to(self.home_angles, 50, False)
                if arrived:
                    # open gripper
                    self.mc.set_gripper_state(0, 50)
                    self.stop_wait(2)
                    self.mc.set_color(0, 255, 0)
                    print('Moved to home position.')

        # start thread
        # end current thread
        if self.current_thread:
            self.current_thread.join()
        # start thread
        self.current_thread = threading.Thread(target=move_home)
        self.current_thread.start()

    def move_cobot_to(self, pose, speed=50, coords=True):
        """Move the robot to a specific pose.
        Args:
            pose: (list) The pose to move to.
            speed: (int) The speed of the movement.
            coords: (bool) Whether to use coordinates or angles.
            Returns: (bool) True if the robot reached the pose, False otherwise.
                """
        print('Moving to:', pose)
        if coords:
            self.mc.send_coords(pose, speed, 0)
        else:
            self.mc.send_angles(pose, speed)

        start = time.time()
        self.mc.set_color(255, 255, 0)
        while True:
            if self.mc.is_in_position(pose, int(coords)):
                self.mc.set_color(0, 255, 0)
                return True
            if time.time() - start > 15:
                print('Timeout')
                self.mc.set_color(255, 0, 0)
                return False

    def go_to_cube(self):
        def detect_cube_and_follow():
            frame = cv2_rec.cap_frame()
            y, x, rot = cv2_rec.aruco_detect(frame)[0]
            print('Cube detected at:', x, y, rot)
            x /= 2.461  # ratio offset
            y /= 2.355  # ratio offset

            speed = self.speed_spin_box.value()
            x_offset = self.x_offset_spin_box.value()
            y_offset = self.y_offset_spin_box.value()
            rot_offset = self.rot_offset_spin_box.value()
            rot = rot + rot_offset
            # normalize angle
            rot = (rot + 180) % 360 - 180  # this step should not be necessary
            # move to the cube
            rot = self.home_coords[5] + (self.home_coords[5] - abs(rot))
            print()
            if self.mc:
                coords = self.home_coords.copy()
                coords[0] = self.home_coords[0] + x + x_offset
                coords[1] = self.home_coords[1] + y + y_offset

                arrived = self.move_cobot_to(coords, speed, True)
                self.stop_wait(1)
                if arrived:
                    coords[2] = 100
                    arrived = self.move_cobot_to(coords, speed, True)
                    self.stop_wait(1)
                    self.mc.send_coord(6, rot, 50)
                    self.stop_wait(2)
                    if arrived:
                        coords[2] = 80
                        coords[5] = rot
                        arrived = self.move_cobot_to(coords, 20, True)
                    # if arrived:
                    #     self.mc.send_coord(6, rot, 50)
                    #     time.sleep(2)

        # end current thread
        if self.current_thread:
            self.current_thread.join()
        # start thread
        self.current_thread = threading.Thread(target=detect_cube_and_follow)
        self.current_thread.start()

    def release_servos(self):
        self.stop_wait(1)
        # set atom to blue color
        self.mc.set_color(0, 0, 255)
        if self.mc:
            self.mc.release_all_servos()
            print('Released all servos.')

    def play_commands(self):
        def run_commands():
            speed = self.speed_spin_box.value()
            # Play commands from the list
            print('Playing commands...')
            if self.mc:
                # set atom to red color
                self.mc.set_color(255, 165, 0)
                for command in self.commands:
                    if command[0] == 'angle':

                        print(f'Going to position: {command[1]}')
                        self.move_cobot_to(command[1], speed, False)
                    elif command[0] == 'gripper':
                        if command[1] == 'close':
                            print('Closing gripper')
                            self.mc.set_gripper_state(1, speed)

                        elif command[1] == 'open':
                            print('Opening gripper')
                            self.mc.set_gripper_state(0, speed)

                        self.stop_wait(2)
                    else:
                        print('Unknown command')
            print('Commands played.')
            # set atom to green color
            self.mc.set_color(0, 255, 0)

        # Create a thread to run the commands
        # wait for the current thread to finish
        # end current thread
        if self.current_thread:
            self.current_thread.join()
        # start thread
        self.current_thread = threading.Thread(target=run_commands())
        self.current_thread.start()

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

        def record_angles():
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
                    self.stop_wait(0.5)

        # end current thread
        if self.current_thread:
            self.current_thread.join()
        # start thread
        self.current_thread = threading.Thread(target=record_angles)
        self.current_thread.start()

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
            current_coords = self.mc.get_coords()
            self.angle_label.setText(f'Current Angles: {current_angles}')
            self.coord_label.setText(f'Current Coords: {current_coords}')

            # print(f'Updated Angles: {current_angles}')  # Optional, for debugging
            # print(f'Updated Coords: {current_coords}')  # Optional, for debugging

    def stop_wait(self, t):
        """Refresh the software screen in real time during the robot movement"""
        if t * 10 <= 1:
            t = 1
        else:
            t = int(t * 10)

        for i in range(1, t + 1):
            QApplication.processEvents()
            time.sleep(0.1)

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
