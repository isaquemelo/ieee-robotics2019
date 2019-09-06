#!/usr/bin/env python3

import ev3dev.ev3 as ev3
from assets.classes.duo import Duo
from datetime import datetime, timedelta
import time
import math

DEFAULT_SPEED = 400


def map_values(n, start1, stop1, start2, stop2):
    return ((n - start1) / (stop1 - start1)) * (stop2 - start2) + start2


class PipeLineRobot:

    ev3.Sound.speak("Robot started...")

    def __init__(self):

        self.pipe = None  # (<size>, <position>)

        # SENSORS
        self.sensor_for_pipeline_following = ev3.InfraredSensor('in4')
        self.frontal_infra = ev3.InfraredSensor('in3')
        self.gyroscope_sensor = ev3.GyroSensor('in2')
        # SENSORS

        # MOTORS
        self.motors = Duo(ev3.LargeMotor('outB'), ev3.LargeMotor('outC'))
        # MOTORS

    # GETTERS
    # - sensors
    def get_side_dist_pipeline_flw(self):
        return self.sensor_for_pipeline_following.value()

    def get_front_dist_pipeline_flw(self):
        return self.frontal_infra.value()

    def get_angle(self):
        return self.gyroscope_sensor.value()
    # GETTERS

    # SETTERS
    # - sensors
    def reset_gyro(self):
        self.gyroscope_sensor.mode = "GYRO-RATE"
        self.gyroscope_sensor.mode = "GYRO-ANG"

    # - motors
    def stop_motors(self):
        self.motors.left.stop()
        self.motors.right.stop()
    # SETTERS

    def move_timed(self, how_long=0.3, direction="forward", speed=DEFAULT_SPEED):
        end_time = datetime.now() + timedelta(seconds=how_long)

        vel = speed

        if direction != "forward":
            vel = -speed

        while datetime.now() < end_time:
            self.motors.left.run_forever(speed_sp=vel), self.motors.right.run_forever(speed_sp=vel)
        self.motors.left.stop(), self.motors.right.stop()

    def rotate(self, angle, axis="own", speed=DEFAULT_SPEED):
        if angle == 0:
            print("Rotate 0 deg does not make sense")
            return

        if 30 > angle > 0:
            speed = map_values(math.fabs(angle), 0, 90, 100, 1000)

        reverse = False
        if angle < 0:
            reverse = True
            angle *= -1

        self.reset_gyro()

        start_angle = self.get_angle()
        # print("start_angle:", start_angle)
        now_angle = start_angle

        self.stop_motors()

        while now_angle < angle + start_angle:
            # print("now angle:", now_angle, "goal: |", angle + start_angle, "|")

            if reverse:
                if axis == "own":
                    self.motors.left.run_forever(speed_sp=-speed)
                    self.motors.right.run_forever(speed_sp=speed)
                else:
                    self.motors.right.run_forever(speed_sp=speed)

                now_angle = self.get_angle() * -1
            else:
                if axis == "own":
                    self.motors.left.run_forever(speed_sp=speed)
                    self.motors.right.run_forever(speed_sp=-speed)
                else:
                    self.motors.left.run_forever(speed_sp=speed)
                now_angle = self.get_angle()

        self.stop_motors()

        self.reset_gyro()
