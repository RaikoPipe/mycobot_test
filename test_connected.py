from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle


mc = MyCobot('COM3', 115200)
print(mc.is_controller_connected())
