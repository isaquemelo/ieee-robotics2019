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
        # self.gyro_value = 0

        self.color_sensors = (2, 2)

        self.ultrasonic_sensors = {"right": ev3.UltrasonicSensor('in2'), "top": 0,
                                   "front-right": ev3.UltrasonicSensor('in4'), "front-left": ev3.UltrasonicSensor('in3')}

        self.infrared_sensors = {"upper_front": 25, "left": ev3.InfraredSensor('in1')}

        # define motors
        self.motors = Duo(ev3.LargeMotor('outA'), ev3.LargeMotor('outB'))
        self.motors.left.polarity = "inversed"
        self.motors.right.polarity = "inversed"

        self.handler = Duo(ev3.MediumMotor('outC'), ev3.LargeMotor('outD'))

        # define status
        self.historic = [""]

    def reset_gyroscope(self):
        self.gyroscope_sensor.mode = 'GYRO-RATE'
        self.gyroscope_sensor.mode = 'GYRO-ANG'

    def get_sensor_data(self, sensor_name):
        # returns the value of a sensor

        if sensor_name == "InfraredSensor":
            return self.ultrasonic_sensors["front-left"].value() / 10, self.ultrasonic_sensors["front-right"].value() / 10, \
                   self.infrared_sensors["upper_front"]

        elif sensor_name == "GyroSensor":
            return self.gyroscope_sensor.angle

        elif sensor_name == "Ultrasonic":
            #return self.ultrasonic_sensor.value() / 10
            return [self.ultrasonic_sensors['left'].value(), self.ultrasonic_sensors['right'].value()]

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

            return [dict_colors[self.color_sensors[0]], dict_colors[self.color_sensors[1]]]

    def rotate(self, angle, speed=DEFAULT_SPEED):
        if angle == -90:
            self.motors.left.run_to_rel_pos(position_sp=-200, speed_sp=speed)
            self.motors.right.run_to_rel_pos(position_sp=200, speed_sp=speed)
            self.motors.left.wait_while("running")
            self.motors.right.wait_while("running")
        elif angle == 90:
            self.motors.left.run_to_rel_pos(position_sp=430, speed_sp=speed)
            self.motors.right.run_to_rel_pos(position_sp=-430, speed_sp=speed)
            self.motors.left.wait_while("running")
            self.motors.right.wait_while("running")

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

    def stop_handler(self):
        self.handler.left.stop()
        self.handler.right.stop()

    def reset(self):
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        pass

    def move_handler(self, how_long=7, direction="down", speed=1000):
        end_time = datetime.now() + timedelta(seconds=how_long)

        vel = speed

        if direction != "down":
            vel = -speed

        while datetime.now() < end_time:
            self.handler.left.run_forever(speed_sp=vel)
        self.handler.left.stop()

    # END OF CRUCIAL METHODS

    def pipeline_support_following(self):
        pid = PID(7, 0, 1.2, setpoint=8)

        while True:
            control = pid(self.infrared_sensors['left'].value())

            #
            # if self.ultrasonic_sensors["front-right"].value() < 100 or \
            #         self.ultrasonic_sensors["front-left"].value() < 100:
            #     print("Vai bater")
            #     self.rotate(90)
            # elif self.ultrasonic_sensors["right"].value() > 100 and self.ultrasonic_sensors["left"].value() > 100:
            #     print("Passou lateralmente")
            #     time.sleep(1.4)
            #     self.rotate(-90)
            #     self.motors.left.run_forever(speed_sp=400)
            #     self.motors.right.run_forever(speed_sp=400)
            #
            #     while not (self.ultrasonic_sensors["right"].value() < 100 and self.ultrasonic_sensors["left"].value() < 100):
            #         pass

            calculated_speed_a = control + DEFAULT_SPEED
            calculated_speed_b = - control + DEFAULT_SPEED

            if calculated_speed_a > 1000:
                calculated_speed_a = 1000
            elif calculated_speed_a < -1000:
                calculated_speed_a = -1000

            if calculated_speed_b > 1000:
                calculated_speed_b = 1000
            elif calculated_speed_b < -1000:
                calculated_speed_b = -1000

            self.motors.left.run_forever(speed_sp=calculated_speed_a)
            self.motors.right.run_forever(speed_sp=calculated_speed_b)

    def request_area(self, area):
        pass

    def request_pipe(self, size):
        pass

    def pipe_rescue(self, size):
        # step size
        if size == 20:
            step_size = 6
            hw_many_cycles = 10
        elif size == 15:
            step_size = 3
            hw_many_cycles = 20
        elif size == 10:
            step_size = 1
            hw_many_cycles = 55

        # data for debugging
        sensor_data = [[], []]
        value_index = 300
        value_minor = 30000

        cycles = 0
        while True:
            self.move_metered(cm=step_size, speed=-DEFAULT_SPEED)
            sensor_values = (self.infrared_sensors['left'].value(), self.ultrasonic_sensors['right'].value())

            sensor_data[0].append(sensor_values[0])
            sensor_data[1].append(sensor_values[1])

            cycles += 1

            if (sensor_values[0] + sensor_values[1]) < value_minor:
                value_minor = sensor_values[0] + sensor_values[1]
                value_index = cycles

            if cycles >= hw_many_cycles:
                break

        counter = 0

        while counter <= hw_many_cycles - value_index:
            self.move_metered(cm=-step_size, speed=-DEFAULT_SPEED)
            counter += 1

        # print(sensor_data)
        self.rotate(-90)

    def get_in_position_to_grab_pipe(self):
        # get close to the pipe with PID
        pid = PID(54, 0, 25, setpoint=0)
        default = 200
        max_speed_bound = 500
        max_control = max_speed_bound - default
        min_control = -max_speed_bound + default
        # baseado no cano o max approach tera que mudar --- no cano de 20 1 é ideal
        max_approach_k = 4
        first_key = False
        second_key = False
        k_to_identify_perpendicular = 5

        while True:

            lis = self.get_sensor_data("InfraredSensor")
            left = lis[0]
            right = lis[1]
            upper_front = lis[2]
            print(lis)

            if left <= max_approach_k or right <= max_approach_k or upper_front <= max_approach_k:
                first_key = True
                if abs(left - right) > k_to_identify_perpendicular and upper_front < 15:
                    print("its perpendicular")
                    break

            if first_key and abs(left - right) <= 1 and min(left, right) < 5:
                second_key = True

            if first_key and second_key:
                print("its parallel")
                break

            control = pid(left - right)

            if control > max_control:
                control = max_speed_bound - default
            elif control < min_control:
                control = -max_speed_bound + default

            self.motors.left.run_forever(speed_sp=default - control)
            self.motors.right.run_forever(speed_sp=default + control)

        self.stop_motors()

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






