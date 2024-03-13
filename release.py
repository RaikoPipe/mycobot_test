from pymycobot.mycobot import MyCobot

mc = MyCobot("COM3", 115200)
mc.release_all_servos()