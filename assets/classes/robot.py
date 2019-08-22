#!/usr/bin/env python3
from assets.classes.duo import Duo
import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
from simple_pid import PID
import json
import time

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
        # self.gyroscope_sensor = ev3.GyroSensor('in4')
        # self.reset_gyroscope()
        self.gyro_value = 0

        self.color_sensors = (0, 0)

        self.ultrasonic_sensors = {"right": ev3.UltrasonicSensor('in2'), "left": ev3.UltrasonicSensor('in1')}

        self.infrared_sensors = {"right": ev3.InfraredSensor('in4'), "left": ev3.InfraredSensor('in3')}

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

    def pipe_rescue(self):
        # step size
        step_size = 6
        full_block_size = 84
        hw_many_cycles = 10 # full_block_size // step_size

        # data for debugging
        sensor_data = [[], []]
        value_index = [300, 300]
        value_minor = [300000, 300000]

        cycles = 0
        while True:
            self.move_metered(cm=step_size, speed=-DEFAULT_SPEED)
            sensor_values = (self.ultrasonic_sensors['left'].value(), self.ultrasonic_sensors['right'].value())

            sensor_data[0].append(sensor_values[0])
            sensor_data[1].append(sensor_values[1])

            cycles += 1

            if sensor_values[0] < value_minor[0]:
                value_minor[0] = self.ultrasonic_sensors['left'].value()
                value_index[0] = cycles

            if sensor_values[1] < value_minor[1]:
                value_minor[1] = self.ultrasonic_sensors['right'].value()
                value_index[1] = cycles

            if cycles >= hw_many_cycles:
                break

        counter = 0
        hw_far_should_i_go = hw_many_cycles - value_index[1]


        print(value_index, value_minor)
        while counter <= hw_many_cycles - value_index[1]:
            self.move_metered(cm=-step_size, speed=-DEFAULT_SPEED)
            counter += 1

        print(sensor_data)


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






