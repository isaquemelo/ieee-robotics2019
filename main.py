#!/usr/bin/env python3

import ev3dev.ev3 as ev3
# import math
from datetime import datetime, timedelta
from time import sleep
import random
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
        while not robot.start:
            robot.btn.process()
            print('PRESS UP TO START')

        robot.initial_location_reset()
        # robot.stop_handler_brake() it should happen

        slope_side = "right"

        while True:
            robot.status = robot.status_dictionary["want10pipe"]
            robot.publish_data()

            while True:
                print("robot.robot_status", robot.robot_status)
                now_state = robot.robot_status
                if now_state in ["pipeInPositionToRescue-10", "pipeInPositionToRescue-15", "pipeInPositionToRescue-20"]:
                    robot.go_grab_pipe_routine(side=slope_side, pipe_being_taken=now_state)
                    robot.robot_status = None
                    break

            pipe_check = robot.still_have_pipe()
            print("do i have pipe?", pipe_check)
            if pipe_check:
                robot.status = robot.status_dictionary["rescuedPipe"]
                robot.publish_data()

                robot.pipeline_support_conection_meeting_area("to pipeline")
                robot.pipeline_support_following()
                still_have_pipe = robot.still_have_pipe()

                while still_have_pipe is True or robot.placed_pipe is False:
                    robot.pipeline_support_conection_meeting_area(side="to meeting area")
                    robot.rotate(angle=90, speed=90)
                    robot.slope_following()
                    robot.rotate(angle=90, speed=90)
                    robot.pipeline_support_conection_meeting_area(side="to pipeline")
                    robot.pipeline_support_following()
                robot.pipeline_support_conection_meeting_area("to meeting area")

                slope_side = "left"

                robot.robot_status = None

            else:
                print("RUNNING ACTION!!! I DONT HAVE THE PIPE")
                robot.current_pipe_size = None
                slope_side = "right"
                robot.move_timed(1.3, speed=150)
                robot.rotate(-90, speed=150)
                robot.robot_status = None
                continue

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
