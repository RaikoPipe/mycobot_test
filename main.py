from pymycobot import MyCobotSocket

# Initialize the MyCobot object with the appropriate port
# Replace 'your_port_here' with the actual serial port
mc = MyCobotSocket('141.44.152.231', 9000)

# Define the zero configuration angles
zero_angles = [0, 0, 0, 0, 0, 0]

# Send the robot to the zero configuration
mc.send_angles(zero_angles, 50)  # 50 is the speed

mc.close()