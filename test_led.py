from pymycobot import MyCobotSocket

# Initialize the MyCobot object with the appropriate port
# Replace 'your_port_here' with the actual serial port
mc = MyCobotSocket('141.44.152.231', 9000)

# Turn off the LEDs
# Note: The method to turn off the LEDs may vary based on your model
# This is a common method used for many models
mc.set_color(0, 0, 0)  # Setting the color to black (0, 0, 0) effectively turns off the LEDs
