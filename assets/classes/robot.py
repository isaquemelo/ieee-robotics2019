#!/usr/bin/env python3
from assets.classes.duo import Duo
import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
from simple_pid import PID
import json

DEFAULT_SPEED = 400


def map_values(n, start1, stop1, start2, stop2):
    return ((n - start1) / (stop1 - start1)) * (stop2 - start2) + start2


class Robot:

    ev3.Sound.speak("Robot started...")

    # CRUCIAL METHODS
    # DO NOT UPDATE
    def __init__(self):
        self.DEFAULT_SPEED = 400

        self.pipe = None

        # define sensors
        self.gyroscope_sensor = ev3.GyroSensor('in4')
        self.reset_gyroscope()

        self.color_sensors = (0, 0)

        self.ultrasonic_sensors = 0

        self.infrared_sensors = {"right": ev3.InfraredSensor('in3'), "left": ev3.InfraredSensor('in2')}

        # define motors
        self.motors = Duo(ev3.LargeMotor('outA'), ev3.LargeMotor('outB'))
        self.motors.left.polarity = "inversed"
        self.motors.right.polarity = "inversed"

        self.handler = Duo(ev3.LargeMotor('outC'), ev3.LargeMotor('outD'))

        # define status
        self.historic = [""]

    def reset_gyroscope(self):
        self.gyroscope_sensor.mode = 'GYRO-RATE'
        self.gyroscope_sensor.mode = 'GYRO-ANG'

    def get_sensor_data(self, sensor_name):
        # returns the value of a sensor

        if sensor_name == "InfraredSensor":
            return self.infrared_sensors.left.value(), self.infrared_sensors.right.value()

        elif sensor_name == "GyroSensor":
            return self.gyroscope_sensor.angle

        elif sensor_name == "Ultrasonic":
            return self.ultrasonic_sensor.value() / 10

        elif sensor_name == "ColorSensor":
            dict_colors = {
                0: 'Undefined',
                1: 'Black',
                2: 'Blue',
                3: 'Green',
                4: 'Yellow',
                5: 'Red',
                6: 'White',
                7: 'Brown'
            }

            return [dict_colors[self.color_sensors.left.color], dict_colors[self.color_sensors.right.color]]

    def rotate(self, angle, axis="own", speed=DEFAULT_SPEED, infrared_sensor_condicional=False, k_for_infrared_sensor=None):

        if 30 > angle > 0:
            speed = map_values(math.fabs(angle), 0, 90, 100, 1000)

        reverse = False

        if angle < 0:
            reverse = True
            angle = angle * -1

        self.reset_gyroscope()

        start_angle = self.get_sensor_data('GyroSensor')
        now_angle = start_angle

        self.motors.left.stop()
        self.motors.right.stop()

        if not infrared_sensor_condicional:
            while now_angle < angle + start_angle:
                if reverse:
                    if axis == "own":
                        self.motors.left.run_forever(speed_sp=-speed)
                        self.motors.right.run_forever(speed_sp=speed)
                    else:
                        self.motors.right.run_forever(speed_sp=speed)

                    now_angle = self.gyroscope_sensor.angle * -1
                else:
                    if axis == "own":
                        self.motors.left.run_forever(speed_sp=speed)
                        self.motors.right.run_forever(speed_sp=-speed)
                    else:
                        self.motors.left.run_forever(speed_sp=speed)
                    now_angle = self.gyroscope_sensor.angle

        elif infrared_sensor_condicional:
            while now_angle < angle + start_angle:

                if self.infrared_sensors["left"].value() <= k_for_infrared_sensor or \
                   self.infrared_sensors["right"].value() <= k_for_infrared_sensor:

                    self.motors.left.stop()
                    self.motors.right.stop()
                    self.reset_gyroscope()
                    return True

                if reverse:
                    if axis == "own":
                        self.motors.left.run_forever(speed_sp=-speed)
                        self.motors.right.run_forever(speed_sp=speed)
                    else:
                        self.motors.right.run_forever(speed_sp=speed)

                    now_angle = self.gyroscope_sensor.angle * -1
                else:
                    if axis == "own":
                        self.motors.left.run_forever(speed_sp=speed)
                        self.motors.right.run_forever(speed_sp=-speed)
                    else:
                        self.motors.left.run_forever(speed_sp=speed)
                    now_angle = self.gyroscope_sensor.angle

        self.motors.left.stop()
        self.motors.right.stop()

        self.reset_gyroscope()
        return False

    def move_timed(self, how_long=0.3, direction="forward", speed=DEFAULT_SPEED):
        end_time = datetime.now() + timedelta(seconds=how_long)

        vel = speed

        if direction != "forward":
            vel = -speed

        while datetime.now() < end_time:
            self.motors.left.run_forever(speed_sp=vel), self.motors.right.run_forever(speed_sp=vel)
        self.motors.left.stop(), self.motors.right.stop()

    def move_metered(self, cm, speed=DEFAULT_SPEED):
        const = 500/17
        self.motors.left.run_to_rel_pos(position_sp=cm * const, speed_sp=speed, stop_action="brake")
        self.motors.right.run_to_rel_pos(position_sp=cm * const, speed_sp=speed, stop_action="brake")
        self.motors.left.wait_while("running")
        self.motors.right.wait_while("running")

    def run_action(self, direction):
        if direction == "forward":
            pass
        elif direction == "left":
            self.rotate(90, axis="own")
        elif direction == "right":
            self.rotate(-90, axis="own")

    def stop_motors(self):
        self.motors.left.stop()
        self.motors.right.stop()

    def reset(self):
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        pass

    def __str__(self) -> str:

        return "ultrasonic: " + str(self.ultrasonic_sensor) + " color: " + str(self.color_sensors) + \
               " infrared: " + self.get_sensor_data("ColorSensor")

    # END OF CRUCIAL METHODS

    def pipe_following(self):
        pass

    def request_area(self, area):
        pass

    def request_pipe(self, size):
        pass

    # def pipe_rescue(self):
    #     sensor_historic = []
    #
    #     # go to color
    #
    #     # go to color
    #
    #     # scanning
    #     c = 0
    #     while True:
    #         if c > 100:
    #             break
    #
    #         print(c)
    #
    #         sensor_historic.append(ev3.UltrasonicSensor("in2").value())
    #         c += 1
    #
    #         self.motors.left.run_forever(speed_sp=300)
    #         self.motors.right.run_forever(speed_sp=300)
    #
    #
    #     self.stop_motors()
    #     print("sensor_historic = ", sensor_historic)
    #     # scanning
    #
    #     # get close to the pipe
    #     # get close to the pipe
    #
    #
    #     pass

    def pipe_rescue(self):
        # keep in mind this function thinks the robot its on the middle of the pipes yield

        # scanning for a pipe
        k = 35
        while True:

            if not self.rotate(angle=90, speed=300, infrared_sensor_condicional=True, k_for_infrared_sensor=k):
                if not self.rotate(angle=-180, speed=300, infrared_sensor_condicional=True, k_for_infrared_sensor=k):
                    if not self.rotate(angle=90, speed=300, infrared_sensor_condicional=True, k_for_infrared_sensor=k):
                        pass
                    else:
                        break
                else:
                    break
            else:
                break

            self.move_metered(cm=5)
            self.stop_motors()
        # scanning for a pipe

        # get close to the pipe with PID
        pid = PID(20, 0, 0, setpoint=0)
        default = 100
        max_speed_bound = 300
        max_control = max_speed_bound - default
        min_control = -max_speed_bound + default
        left_value = self.infrared_sensors["left"].value()
        right_value = self.infrared_sensors["right"].value()
        approach_k = 1
        while left_value > approach_k and right_value > approach_k:

            left_value = self.infrared_sensors["left"].value()
            right_value = self.infrared_sensors["right"].value()

            control = abs(pid(left_value - right_value))

            if control > max_control:
                control = max_speed_bound - default
            elif control < min_control:
                control = -max_speed_bound + default

            if left_value > right_value:
                self.motors.left.run_forever(speed_sp=default + control)
                self.motors.right.run_forever(speed_sp=default - control)

            elif right_value > left_value:
                self.motors.left.run_forever(speed_sp=default - control)
                self.motors.right.run_forever(speed_sp=default + control)

        self.stop_motors()
        # get close to the pipe with PID


        pass

    def anti_falling(self):
        pass

    def color_alignment(self):
        pass

    def pid_alignment(self):
        pass

    def gap_fill(self):
        pass

    def gap_measurement(self):
        pass






