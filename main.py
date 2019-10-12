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
    while True:
        robot.slope_following()
        sleep(7)
    try:
        print(robot.still_have_pipe())
        robot.initial_location_reset()
        robot.status = robot.status_dictionary["doneInitialPositionReset"]
        robot.publish_data()

        slope_side = "right"

        while True:
            robot.status = robot.status_dictionary["want10pipe"]
            robot.publish_data()

            while True:
                print(robot.robot_status)
                if robot.robot_status in ["pipeInPositionToRescue-10", "pipeInPositionToRescue-15", "pipeInPositionToRescue-20"]:
                    robot.go_grab_pipe_routine(side=slope_side, pipe_being_taken=robot.robot_status)
                    break

            if robot.still_have_pipe():
                robot.status = robot.status_dictionary["rescuedPipe"]
                robot.publish_data()

                robot.pipeline_support_conection_meeting_area("left")
                robot.pipeline_support_following()
                robot.pipeline_support_conection_meeting_area("right")

                slope_side = "left"

            else:
                robot.status = robot.status_dictionary["want10pipe"]
                robot.publish_data()
                continue

            break


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
