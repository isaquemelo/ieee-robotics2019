#!/usr/bin/env python3

import ev3dev.ev3 as ev3
# import math
from datetime import datetime, timedelta
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
    # robot.pipeline_support_conection_meeting_area("to pipeline")
    # exit(0)

    try:
        robot.pipeline_support_following()
        # print(robot.still_have_pipe())
        # robot.initial_location_reset()
        # robot.go_grab_pipe_routine(side="right", pipe_being_taken="pipeInPositionToRescue-15")
        # robot.pipeline_support_conection_meeting_area("to pipeline")
        # robot.pipeline_support_following()
        # robot.pipeline_support_conection_meeting_area("to meeting area")

        # slope_side = "right"
        #
        # while True:
        #     robot.status = robot.status_dictionary["want10pipe"]
        #     robot.publish_data()
        #
        #     while True:
        #         print("robot.robot_status", robot.robot_status)
        #         now_state = robot.robot_status
        #         if now_state in ["pipeInPositionToRescue-10", "pipeInPositionToRescue-15", "pipeInPositionToRescue-20"]:
        #             robot.go_grab_pipe_routine(side=slope_side, pipe_being_taken=now_state)
        #             break
        #
        #     if robot.still_have_pipe():
        #         robot.status = robot.status_dictionary["rescuedPipe"]
        #         robot.publish_data()
        #
        #         robot.status = robot.status_dictionary["want10pipe"]
        #         robot.publish_data()
        #
        #         robot.pipeline_support_conection_meeting_area("to pipeline")
        #         robot.pipeline_support_following()
        #         robot.pipeline_support_conection_meeting_area("to meeting area")
        #
        #         slope_side = "left"
        #
        #     else:
        #         # robot.status = robot.status_dictionary["want10pipe"]
        #         # robot.publish_data()
        #         slope_side = "left"
        #         continue

    except KeyboardInterrupt:
        robot.motors.right.stop()
        robot.motors.left.stop()
        robot.handler.left.reset()
        robot.handler.left.stop()
        robot.handler.left.reset()
        robot.handler.left.stop()

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
    robot.handler.left.reset()
    robot.handler.left.stop()

    server.client.loop_stop()
    server.client.disconnect()
