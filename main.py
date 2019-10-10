#!/usr/bin/env python3

import ev3dev.ev3 as ev3
# import math
# from datetime import datetime, timedelta
from time import sleep

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
        # while True:
        #     robot.initial_location_reset()
        #     robot.status = robot.status_dictionary["doneInitialPositionReset"]
        #     robot.publish_data()
        #
        #     robot.status = robot.status_dictionary["want10pipe"]
        #     robot.publish_data()
        #
        #     while robot.robot_status != "pipeInPositionToRescue-10": print(robot.robot_status)
        #     robot.go_grab_pipe_routine(side="right")
        #     robot.status = robot.status_dictionary["rescuedPipe"]
        #     robot.publish_data()
        #
        #     robot.status = robot.status_dictionary["want10pipe"]
        #     robot.publish_data()
        #
        #     robot.pipeline_support_diving()
        #     robot.pipeline_support_following()
        #
        #     break
        # robot.black_line_following("right")
        # robot.initial_location_reset()
        # robot.pipeline_support_diving()
        robot.pipeline_support_following()
        robot.pipeline_support_conection_meeting_area("right")

    except KeyboardInterrupt:
        robot.motors.right.stop()
        robot.motors.left.stop()
        robot.handler.left.reset()
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
