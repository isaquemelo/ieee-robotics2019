#!/usr/bin/env python3

import ev3dev.ev3 as ev3
from simple_pid import PID
from assets.classes.PipeLineRobot import PipeLineRobot


# 15.6 0 4.8
#pid = PID(15.6, 0, 4.8, setpoint=-4)

robot = PipeLineRobot()

def main():
    try:
        # it may need to get adjusted according to the value that the material from pipeline base reflects to the infrared
        setpoint = 4
        default_speed = 400
        pid = PID(12, 0, 2, setpoint=setpoint)
        side_k_to_rotate = 15
        front_k_to_rotate = 5
        rotation_speed = 40

        speed_a = 0
        speed_b = 0
        while True:
            side_distance = robot.get_side_dist_pipeline_flw()
            front_distance = robot.get_front_dist_pipeline_flw()
            control = pid(side_distance)
            # print(side_distance)

            if side_distance > side_k_to_rotate:
                robot.stop_motors()
                robot.move_timed(0.7, speed=-1000)
                robot.rotate(-90, speed=rotation_speed)

            if front_distance < front_k_to_rotate:
                robot.stop_motors()
                robot.rotate(90, speed=rotation_speed)

            speed_a = control - default_speed
            speed_b = -control - default_speed

            if speed_a >= 1000:
                speed_a = 1000
            elif speed_a <= -1000:
                speed_a = -1000

            if speed_b >= 1000:
                speed_b = 1000
            elif speed_b <= -1000:
                speed_b = -1000

            robot.motors.left.run_forever(speed_sp=speed_a)
            robot.motors.right.run_forever(speed_sp=speed_b)

    except KeyboardInterrupt:
        robot.stop_motors()


try:
    if __name__ == '__main__':
        main()

except KeyboardInterrupt:
    robot.stop_motors()
