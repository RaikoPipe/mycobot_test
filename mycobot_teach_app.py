import sys

import numpy as np
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
        self.home_angles = [2.81, 32.87, -98.78, -23.29, -1.58, 139.39]
        self.home_coords = [52.9, -62.6, 217.8, 178.98, 1.95, 132.06]
        self.bin_a_angles = [68.37, -21.53, -78.75, 11.07, -0.35, -128]
        self.bin_b_angles = [103.27, -5.71, -98.78, 19.24, 1.49, 150.38]
        self.bin_c_angles = [-17, -57, -36, 17, -6, 163]
        self.bin_d_angles = [-31, -20, -94, 32, 1, 142]
        self.loading_area_angles = [49, -73, -1, -2, -2, -116]
        self.id_to_angles = {
            0: self.bin_a_angles,
            1: self.bin_b_angles,
            2: self.bin_c_angles,
            3: self.bin_d_angles,
            4: self.loading_area_angles
        }
        self.pick_up_angles = [20.21, 13.71, -102.04, 2.19, -0.26, 159.16]
        self.stop_record_flag = False
        self.commands = []  # This list will hold the saved commands
        self.init_ui()

        # self.timer = QtCore.QTimer(self)  # Initialize the timer
        # self.timer.setInterval(2000)  # Set the interval to 1000 milliseconds (1 second)
        # self.timer.timeout.connect(self.update_angles)  # Connect the timeout signal to the update_angles method

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
        self.speed_spin_box.setValue(75)  # Default value
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
        self.y_offset_spin_box.setValue(-6)
        layout.addWidget(self.y_offset_spin_box)

        # Create a QLabel for the QSpinBox
        self.spin_box_label_rot_offset = QLabel('Rot Offset:')
        layout.addWidget(self.spin_box_label_rot_offset)

        self.rot_offset_spin_box = QSpinBox()
        self.rot_offset_spin_box.setMinimum(-45)
        self.rot_offset_spin_box.setMaximum(45)
        self.rot_offset_spin_box.setValue(0)
        layout.addWidget(self.rot_offset_spin_box)

        # add red stop button todo: implement
        self.stop_button = QPushButton('STOP')
        self.stop_button.clicked.connect(self.emergency_stop)
        # layout.addWidget(self.stop_button)

        # Exit button
        self.exit_button = QPushButton('Exit')
        self.exit_button.clicked.connect(self.close_app)  # Connect the button to close_app method
        layout.addWidget(self.exit_button)

        self.central_widget.setLayout(layout)

    def connect_robot(self):
        # Connect to the MyCobot
        self.mc = MyCobotSocket('141.44.152.231', 9000)
        # self.mc = MyCobot('/dev/ttyTHS1', 1000000)
        print('Connected to robot.')
        # self.timer.start()  # Start the timer

    def disconnect_robot(self):
        if self.mc:
            self.mc.close()
            self.mc = None
            print('Disconnected from robot.')

    def do_no_gesture(self):
        left_no = self.home_angles.copy()
        left_no[0] = 16
        right_no = self.home_angles.copy()
        right_no[0] = -16
        arrived = self.move_cobot_to(left_no, 100, False)
        if arrived:
            arrived = self.move_cobot_to(right_no, 100, False)
            if arrived:
                arrived = self.move_cobot_to(self.home_angles, 100, False)
                if arrived:
                    self.mc.set_color(0, 255, 0)

    def move_home(self):
        if self.mc:
            arrived = self.move_cobot_to(self.home_angles, 50, False)
            if arrived:
                # open gripper
                self.mc.set_gripper_state(0, 50)
                self.stop_wait(2)
                self.mc.set_color(0, 255, 0)
                print('Moved to home position.')

            return arrived

    def go_home(self):

        # start thread
        # end current thread
        if self.current_thread:
            self.current_thread.join()
        # start thread
        self.current_thread = threading.Thread(target=self.move_home)
        self.current_thread.start()

    def move_cobot_to(self, pose, speed=50, coords=True):
        """Move the robot to a specific pose.
        Args:
            pose: (list) The pose to move to.
            speed: (int) The speed of the movement.
            coords: (bool) Whether to use coordinates or angles.
            Returns: (bool) True if the robot reached the pose, False otherwise.
                """

        def is_in_position(desired_pose, coords):
            if coords:
                current_pose = self.mc.get_coords()
            else:
                current_pose = self.mc.get_angles()

            if current_pose is None:
                time.sleep(0.1)
                return is_in_position(desired_pose, coords)
            # if distance is less than 5mm, return True
            # this is a workaround. For some reason the developers didn't consider
            # that the rotation overflows from 180 to -180, so the distance calculation is wrong.
            distance = np.linalg.norm(np.array(desired_pose[:3]) - np.array(current_pose[:3]))
            if distance < 10:
                return True
            return False

        print('Moving to:', pose)

        self.mc.set_color(255, 255, 0)
        if coords:
            self.mc.send_coords(pose, speed, 0)
        else:
            self.mc.send_angles(pose, speed)

        start = time.time()
        while True:
            mc_is_in_position = self.mc.is_in_position(pose, int(coords))
            arrived = is_in_position(pose, coords)
            if arrived == 1:
                return True

            elif time.time() - start > 15:
                print('Timeout. Aborting.')
                self.mc.set_color(255, 0, 0)
                self.move_home()
                self.do_no_gesture()
                return False

            elif time.time() - start > 10:
                print('Cobot has stopped moving. Resending command.')
                # self.mc.set_color(0, 255, 0)
                if coords:
                    self.mc.send_coords(pose, speed, 0)
                else:
                    self.mc.send_angles(pose, speed)
                time.sleep(2)
                continue

            if mc_is_in_position == -1:
                # error
                print('Cobot has had an error.')
                self.mc.set_color(255, 0, 0)
                return True
            # elif time.time() - start > 1:
            #     # check if robot has stopped moving
            #     if self.mc.is_moving() in [0, -1]:
            #         # send the command again
            #         if self.mc.is_moving() == -1:
            #             print('Cobot has had an error.')
            #         if coords:
            #             self.mc.send_coords(pose, speed, 0)
            #         else:
            #             self.mc.send_angles(pose, speed)

    def go_to_cube(self):
        def detect_cube_and_follow():
            frame = cv2_rec.cap_frame()
            poses, ids = cv2_rec.aruco_detect(frame)
            y, x, rot = poses[0]
            bin_id = ids[0]
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
            print(f'Rotating from {self.home_coords[5]} to {rot}')
            if self.mc:
                coords = self.home_coords.copy()
                coords[0] = self.home_coords[0] + x + x_offset
                coords[1] = self.home_coords[1] + y + y_offset

                arrived = self.move_cobot_to(coords, speed, True)
                if arrived:
                    coords[2] = 120
                    arrived = self.move_cobot_to(coords, speed, True)
                    if arrived:
                        grasp_pose = coords.copy()
                        grasp_pose[5] = rot
                        arrived = self.move_cobot_to(grasp_pose, speed, True)
                        # arrived = self.mc.sync_send_coords(grasp_pose, speed, 0)
                        if arrived:
                            coords[2] = 80
                            coords[5] = rot
                            arrived = self.move_cobot_to(coords, speed, True)
                            self.stop_wait(2)
                            if arrived:
                                self.mc.set_gripper_state(1, speed)
                                self.stop_wait(1)
                                arrived = self.move_cobot_to(self.pick_up_angles, speed, False)
                                if arrived:
                                    bin_angles = self.id_to_angles[bin_id]
                                    arrived = self.move_cobot_to(bin_angles, speed, False)
                                if arrived:
                                    self.mc.set_gripper_state(0, speed)
                                    self.stop_wait(1)
                                    arrived = self.move_cobot_to(self.home_angles, speed, False)
                                    if arrived:
                                        self.mc.set_color(0, 255, 0)
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

    def emergency_stop(self):
        if self.mc:
            self.current_thread.join()
            self.mc.set_color(255, 0, 0)
            self.mc.stop()

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
