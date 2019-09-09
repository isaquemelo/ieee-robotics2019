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


class PipeLineRobot:
    ev3.Sound.speak("Robot started...")

    # CRUCIAL METHODS
    # DO NOT UPDATE
    def __init__(self):
        self.DEFAULT_SPEED = 400

        self.pipe = None

        # define sensors
        self.gyroscope_sensor = ev3.GyroSensor('in2')
        self.reset_gyroscope()
        self.gyro_value = 0

        self.color_sensors = (2, 2)

        #self.ultrasonic_sensors = {"right": ev3.UltrasonicSensor('in2'), "top": 0,
                                   #"front-right": ev3.UltrasonicSensor('in4'), "front-left": ev3.UltrasonicSensor('in3')}

        self.infrared_sensors = {"frontal": ev3.InfraredSensor('in3'), "side": ev3.InfraredSensor('in4'), "back": ev3.InfraredSensor('in1')}

        # define motors
        self.motors = Duo(ev3.LargeMotor('outB'), ev3.LargeMotor('outC'))
        self.motors.left.polarity = "inversed"
        self.motors.right.polarity = "inversed"

        self.handler = Duo(ev3.LargeMotor('outA'), ev3.LargeMotor('outA'))

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

    def rotate(self, angle, axis="own", speed=DEFAULT_SPEED):
        if angle == 0:
            print("Rotate 0 deg does not make sense")
            return

        # if 30 > angle > 0:
        #     speed = map_values(math.fabs(angle), 0, 90, 100, 1000)

        reverse = False
        if angle < 0:
            reverse = True
            angle *= -1

        self.reset_gyroscope()

        start_angle = self.gyroscope_sensor.value()
        # print("start_angle:", start_angle)
        now_angle = start_angle

        self.stop_motors()

        while now_angle < angle + start_angle:
            print("now angle:", now_angle, "goal: |", angle + start_angle, "|")

            if reverse:
                if axis == "own":
                    self.motors.left.run_forever(speed_sp=-speed)
                    self.motors.right.run_forever(speed_sp=speed)
                else:
                    self.motors.right.run_forever(speed_sp=speed)

                now_angle = self.gyroscope_sensor.value() * -1
            else:
                if axis == "own":
                    self.motors.left.run_forever(speed_sp=speed)
                    self.motors.right.run_forever(speed_sp=-speed)
                else:
                    self.motors.left.run_forever(speed_sp=speed)
                now_angle = self.gyroscope_sensor.value()
        self.motors.left.stop()
        self.motors.right.stop()

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

    def pipeline_support_following(self):
        default_speed = 350
        pid = PID(12, 0, 2, setpoint=4)
        side_k_to_rotate = 15
        front_k_to_rotate = 5
        rotation_speed = 40

        speed_a = 0
        speed_b = 0

        while True:
            side_distance = self.infrared_sensors['side'].value()
            front_distance = self.infrared_sensors['frontal'].value()
            control = pid(side_distance)

            if side_distance > side_k_to_rotate:
                self.stop_motors()
                self.move_timed(0.9, speed=-500)
                self.rotate(-80, axis="own", speed=90)

            if front_distance < front_k_to_rotate:
                self.stop_motors()
                self.rotate(80, axis="own", speed=90)

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

            self.motors.left.run_forever(speed_sp=speed_a)
            self.motors.right.run_forever(speed_sp=speed_b)

    def initial_location_reset(self):
        # anda frente -> procura cor (verde, preto, undefined)
        # achou cor:
            # alinha
        # se ambos undefined:
          # rezinha
          # 90 graus
          # anda frente ate achar cor
              # achei verde: fim
              # achei preto:
                 # alinha
                 # 90 graus
                 # pid linha preta undefined-undefined
                 # 90 graus
                 # pid undefined ate verde-verde
        # se preto:
          # 90 graus
          # segue preto "PID"
            # achar undefined:
            # 90 graus
            # PID undefined ate achar verde

        # se verde:
          # 180 graus
          # anda ate preto
          # alinha cor
          # 90 graus
          # pid ate undefined-undefined
          # 90 graus
          # pid undefined ate verde-verde

        pass

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