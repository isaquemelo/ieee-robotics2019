#!/usr/bin/env python3

import ev3dev.ev3 as ev3
# import math
# from datetime import datetime, timedelta
# import time

# from simple_pid import PID
from assets.classes.PipeLineRobot import PipeLineRobot
# import json
# from assets.handlers.button import ButtonApproach
# from assets.handlers.undefined_dealing import undefined_dealing
from assets.handlers.server_config import Server

DEFAULT_SPEED = 350


robot = PipeLineRobot()

# 51 lateral]
# 110 frente

def main():
    try:
        #robot.rotate(80, axis="own", speed=90)
        #robot.pipeline_support_following()
        #robot.black_line_following()
        robot.initial_location_reset()
        #robot.underground_position_reset(side="right")
        # while True:
        #     print(robot.get_sensor_data("ColorSensor"))
        #robot.pipe_rescue(15)

        #robot.pipeline_support_following()
        #robot.get_in_position_to_grab_pipe()
            #time.sleep(8)
            # break

            # robot.motors.left.run_forever(speed_sp=speed)
            # robot.motors.right.run_forever(speed_sp=speed)
            # colors = robot.get_sensor_data("ColorSensor")
            # print(colors)
            # if colors[0] != "Blue" and colors[1] != "Blue":
            #     robot.stop_motors()
            #     break

    except KeyboardInterrupt:
        robot.motors.right.stop()
        robot.motors.left.stop()
        robot.handler.left.stop()
        robot.motors.alternative.stop()

        server.client.loop_stop()
        server.client.disconnect()


try:
    if __name__ == '__main__':
        main()

except KeyboardInterrupt:
    print(Exception)
    robot.motors.right.stop()
    robot.motors.left.stop()
    robot.handler.left.stop()
    robot.motors.alternative.stop()

    server.client.loop_stop()
    server.client.disconnect()