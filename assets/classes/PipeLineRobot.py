#!/usr/bin/env python3
from assets.classes.duo import Duo
import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
from simple_pid import PID
# import json
import time

from calibrated_consts import black_line_following

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

        self.color_sensors = (ev3.ColorSensor('in1'), ev3.ColorSensor('in4'))
        self.dict_colors = {
            0: 'Undefined',
            1: 'Black',
            2: 'Blue',
            3: 'Green',
            4: 'Yellow',
            5: 'Red',
            6: 'White',
            7: 'Brown'
        }

        self.ultrasonic_sensors = {"right": 10, "left": 10}
        # "front-right": ev3.UltrasonicSensor('in4'), "front-left": ev3.UltrasonicSensor('in3')}

        self.infrared_sensors = {"diagonal_top": ev3.InfraredSensor('in3'), "left": 10, "front": 10}

        # define motors
        self.motors = Duo(ev3.LargeMotor('outB'), ev3.LargeMotor('outD'))
        self.motors.left.polarity = "inversed"
        self.motors.right.polarity = "inversed"

        self.handler = Duo(ev3.LargeMotor('outA'), ev3.LargeMotor('outA'))
        self.handler.left.run_forever(speed_sp=-150)

        # define status
        self.historic = [""]

    def reset_gyroscope(self):
        self.gyroscope_sensor.mode = 'GYRO-RATE'
        self.gyroscope_sensor.mode = 'GYRO-ANG'

    def get_sensor_data(self, sensor_name, ColorSensorMode="COL-REFLECT"):
        # returns the value of a sensor

        if sensor_name == "InfraredSensor":
            return [self.infrared_sensors["left"], self.infrared_sensors["front"],
                    self.infrared_sensors["diagonal_top"].value()]
            # return self.ultrasonic_sensors["front-left"].value() / 10, self.ultrasonic_sensors[
            #     "front-right"].value() / 10, \
            #        self.infrared_sensors["upper_front"]

        elif sensor_name == "GyroSensor":
            return self.gyroscope_sensor.angle

        elif sensor_name == "Ultrasonic":
            # return self.ultrasonic_sensor.value() / 10
            return [self.ultrasonic_sensors['left'].value(), self.ultrasonic_sensors['right'].value()]

        elif sensor_name == "ColorSensor":
            if ColorSensorMode == "REF-RAW":
                return [self.color_sensors[0].value(), self.color_sensors[1].value()]

            return [self.dict_colors[self.color_sensors[0].color], self.dict_colors[self.color_sensors[1].color]]

    def rotate(self, angle, axis="own", speed=DEFAULT_SPEED):
        print("rotating ", angle, "deg at ", speed, " speed")
        if angle == 0:
            # print("Rotate 0 deg does not make sense")
            return

        # if 30 > angle > 0:
        #     speed = map_values(math.fabs(angle), 0, 90, 100, 1000)

        reverse = False
        if angle < 0:
            reverse = True
            angle *= -1

        self.reset_gyroscope()

        start_angle = self.gyroscope_sensor.value()
#         # print("start_angle:", start_angle)
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
        const = 500 / 17
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

    # def pipeline_support_following(self):
    #     default_speed = 350
    #     pid = PID(12, 0, 2, setpoint=4)
    #     side_k_to_rotate = 15
    #     front_k_to_rotate = 5
    #     rotation_speed = 40
    #
    #     speed_a = 0
    #     speed_b = 0
    #
    #     while True:
    #         side_distance = self.infrared_sensors['side'].value()
    #         front_distance = self.infrared_sensors['frontal'].value()
    #         control = pid(side_distance)
    #
    #         if side_distance > side_k_to_rotate:
    #             self.stop_motors()
    #             self.move_timed(0.9, speed=-500)
    #             self.rotate(-80, axis="own", speed=90)
    #
    #         if front_distance < front_k_to_rotate:
    #             self.stop_motors()
    #             self.rotate(80, axis="own", speed=90)
    #
    #         speed_a = control - default_speed
    #         speed_b = -control - default_speed
    #
    #         if speed_a >= 1000:
    #             speed_a = 1000
    #         elif speed_a <= -1000:
    #             speed_a = -1000
    #
    #         if speed_b >= 1000:
    #             speed_b = 1000
    #         elif speed_b <= -1000:
    #             speed_b = -1000
    #
    #         self.motors.left.run_forever(speed_sp=speed_a)
    #         self.motors.right.run_forever(speed_sp=speed_b)

    def pipeline_support_following(self):
        default_speed = 350
        max_speed = 1000
        min_speed = -1000
        pid = PID(12, 0, 2, setpoint=4)

        side_k_to_rotate = 20
        front_k_to_rotate = 5
        rotation_speed = 40

        speed_a = 0
        speed_b = 0


        while True:
            side_distance = self.infrared_sensors['right']
            front_distance = self.infrared_sensors['front']
            control = pid(side_distance)


            print(side_distance, front_distance)

            if side_distance > side_k_to_rotate:
                self.stop_motors()
                self.move_timed(0.9, speed=500)
                self.rotate(-80, axis="own", speed=90)

            if front_distance < front_k_to_rotate:
                self.stop_motors()
                self.rotate(80, axis="own", speed=90)

            speed_a = default_speed + control
            speed_b = default_speed - control


            if speed_a >= 1000:
                speed_a = 1000
            elif speed_a <= -1000:
                speed_a = -1000

            if speed_b >= 1000:
                speed_b = 1000
            elif speed_b <= -1000:
                speed_b = -1000

            self.motors.left.run_forever(speed_sp=default_speed)
            self.motors.right.run_forever(speed_sp=speed_b)

    def adjust_corner_to_go_green(self):
        self.rotate(angle=-80)
        self.move_timed(how_long=1, direction="backward")
        self.rotate(angle=80)
        self.rotate(angle=80)
        return

    def go_for_green(self):
        self.color_sensors[0].mode = "COL-REFLECT"
        self.color_sensors[1].mode = "COL-REFLECT"

        bord_dist_expected = 30
        default_speed = 200
        max_value = 400
        min_value = -400
        left_bord_pid = PID(2, 0, 1, setpoint=10)

        while True:
            color_data = self.get_sensor_data("ColorSensor")
            left_dist = self.get_sensor_data("InfraredSensor")[0]
            control = left_bord_pid(bord_dist_expected - left_dist)

            if color_data[0] == "Green" and color_data[1] == "Green":
                self.stop_motors()
                return

            l_speed = default_speed - control
            r_speed = default_speed + control

            if l_speed > max_value:
                l_speed = max_value
            elif l_speed < min_value:
                l_speed = min_value

            if r_speed > max_value:
                r_speed = max_value
            elif r_speed < min_value:
                r_speed = min_value

            self.motors.left.run_forever(speed_sp=l_speed)
            self.motors.right.run_forever(speed_sp=r_speed)

    def adjust_before_black_line_flw(self, speed=200):
        self.stop_motors()
        print("making sure the robot its in the correct position before doing black line flw")

        self.color_sensors[0].mode = "REF-RAW"
        self.color_sensors[1].mode = "REF-RAW"

        color_data = self.get_sensor_data("ColorSensor", ColorSensorMode="REF-RAW")

        while color_data[0] < 520:
            self.motors.left.run_forever(speed_sp=speed)
            color_data = self.get_sensor_data("ColorSensor", ColorSensorMode="REF-RAW")
        self.stop_motors()

        while color_data[1] < 520:
            self.motors.right.run_forever(speed_sp=speed)
            color_data = self.get_sensor_data("ColorSensor", ColorSensorMode="REF-RAW")
        self.stop_motors()


        self.color_sensors[0].mode = "COL-REFLECT"
        self.color_sensors[1].mode = "COL-REFLECT"
        return

    def initial_location_reset(self):
        self.stop_motors()

        inner_speed = 400  # > 150
        rotation_speed = 150  # > 50
        while True:
            color_data = self.get_sensor_data("ColorSensor")

            if color_data[0] == "Black" or color_data[1] == "Black":
                print("blackzada")
                self.stop_motors()
                self.color_alignment()
                self.move_timed(0.8, direction="backwards", speed=inner_speed)
                self.rotate(180, speed=2*rotation_speed)
                while True:
                    self.motors.left.run_forever(speed_sp=inner_speed)
                    self.motors.right.run_forever(speed_sp=inner_speed)
                    color_data = self.get_sensor_data("ColorSensor")
                    if "Green" in color_data:
                        #self.color_alignment()
                        self.move_timed(0.3, direction="forward", speed=inner_speed)
                        self.underground_position_reset()
                        break

                        #while self.get_sensor_data("ColorSensor")

            elif color_data[0] == "Green" or color_data[1] == "Green":
                print("verdezada")
                self.stop_motors()
                self.underground_position_reset()
                self.color_alignment("Green")

            elif self.get_sensor_data("InfraredSensor")[2] > 70:
                color_data = self.get_sensor_data("ColorSensor")
                if color_data[0] == "Black":
                    self.stop_motors()
                    self.rotate(angle=20)
                    continue
                if color_data[1] == "Black":
                    self.stop_motors()
                    self.rotate(angle=-20)
                    continue
                print("Abismo")
                while True:
                    self.motors.left.run_forever(speed_sp=200)
                    self.motors.right.run_forever(speed_sp=200)
                    color_data = self.get_sensor_data("ColorSensor")
                    if "Undefined" in color_data:
                        print("Found undefined")
                        self.color_alignment()
                        self.move_timed(0.4, speed=inner_speed, direction="backwards")
                        self.rotate(80, speed=rotation_speed)

                        default_speed = 300
                        set = 20
                        pid = PID(5, 0, 2, setpoint=-20)
                        print("got inside PId")
                        while True:

                            side_distance = self.get_sensor_data("InfraredSensor")[0]
                            color_data = self.get_sensor_data("ColorSensor")
                            upper_dist = self.get_sensor_data("InfraredSensor")[2]
                            control = pid(set - side_distance)

                            # print(side_distance, front_distance)

                            speed_a = default_speed - control
                            speed_b = default_speed + control

                            if speed_a >= 1000:
                                speed_a = 600
                            elif speed_a <= -1000:
                                speed_a = -600

                            if speed_b >= 1000:
                                speed_b = 600
                            elif speed_b <= -1000:
                                speed_b = -600

                            self.motors.left.run_forever(speed_sp=speed_b)
                            self.motors.right.run_forever(speed_sp=speed_a)

                            if "Black" in color_data and upper_dist < 50:
                                self.stop_motors()
                                print("Right side!")
                                self.adjust_before_black_line_flw()
                                #self.color_alignment()
                                self.move_timed(0.8, direction="backwards", speed=inner_speed)
                                self.rotate(160, speed=rotation_speed)

                                while True:
                                    color_data = self.get_sensor_data("ColorSensor")

                                    if "Green" in color_data:
                                        self.color_alignment()
                                        self.underground_position_reset(side="right")
                                        return

                                    self.motors.left.run_forever(speed_sp=DEFAULT_SPEED)
                                    self.motors.right.run_forever(speed_sp=DEFAULT_SPEED)

                            elif "Green" in color_data and upper_dist > 50:
                                self.stop_motors()
                                print("Left side!")
                                # if biggest > 35:
                                self.underground_position_reset(side="left")
                                return
                                # else:
                                #     self.underground_position_reset(side="right")

            self.motors.left.run_forever(speed_sp=250)
            self.motors.right.run_forever(speed_sp=250)
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

    def black_line_routine(self, rotation_speed=150, inner_speed=400):
        self.move_timed(0.5, speed=80)
        self.move_timed(0.3, direction="backwards", speed=80)
        self.color_alignment()
        self.rotate(80, speed=rotation_speed - 50)
        self.black_line_following()  # only returns when both sensors are undefined
        self.move_timed(0.5, direction="backwards", speed=inner_speed)
        self.rotate(90, speed=rotation_speed - 50)

        while True:
            color_data = self.get_sensor_data("ColorSensor")

            self.motors.left.run_forever(speed_sp=inner_speed - 150)
            self.motors.right.run_forever(speed_sp=inner_speed - 150)

            if color_data[0] == "Green" or color_data[1] == "Green":
                self.color_alignment()
                self.stop_motors()
                # print("All set!")
                time.sleep(10)
                break

    def black_line_following(self):
        self.stop_motors()
        print("Black line fowlling")
        self.color_sensors[0].mode = "REF-RAW"
        self.color_sensors[1].mode = "REF-RAW"

        white_value = 410
        setpoint = 20
        upper_dist_const = 60
        default_speed = 200
        max_value = 400
        min_value = -400
        l_pid = PID(0.5, 0, 0.25, setpoint=setpoint)

        while True:
            color_data = self.get_sensor_data("ColorSensor", "REF-RAW")
            left = color_data[0]
            upper_infrared_dist = self.get_sensor_data("InfraredSensor")[2]

            if upper_infrared_dist > upper_dist_const:
                self.stop_motors()
                return

            dif = left - white_value
            control = l_pid(dif)

            l_speed = default_speed - control
            r_speed = default_speed + control

            if l_speed > max_value:
                l_speed = max_value
            elif l_speed < min_value:
                l_speed = min_value

            if r_speed > max_value:
                r_speed = max_value
            elif r_speed < min_value:
                r_speed = min_value

            self.motors.left.run_forever(speed_sp=l_speed)
            self.motors.right.run_forever(speed_sp=r_speed)

    def undefined_following(self):
        default_speed = 300
        reduce_speed = 150

        while True:
            self.motors.left.run_forever(speed_sp=default_speed)
            self.motors.right.run_forever(speed_sp=default_speed)

            color_data = self.get_sensor_data("ColorSensor", ColorSensorMode="REF-RAW")
            if color_data[0] > 500:
                while self.get_sensor_data("ColorSensor", ColorSensorMode="REF-RAW")[0] > 460:
                    self.motors.left.run_forever(speed_sp=default_speed + reduce_speed)
                    self.motors.right.run_forever(speed_sp=default_speed - reduce_speed)

                continue
            if color_data[0] < 460:
                while self.get_sensor_data("ColorSensor", ColorSensorMode="REF-RAW")[0] < 500:
                    self.motors.left.run_forever(speed_sp=default_speed - reduce_speed)
                    self.motors.right.run_forever(speed_sp=default_speed + reduce_speed)

                continue

    # def black_line_following(self):
    #     white_value = black_line_following["white_value"]
    #     max_white_var = black_line_following["max_white_var"]
    #     setpoint = black_line_following["setpoint"]
    #
    #     default_speed = 200
    #     speed_to_get_on_black_line = 30
    #     max_value = 400
    #     min_value = -400
    #     l_pid = PID(2, 0.2, 1, setpoint=setpoint)
    #
    #     self.color_sensors[0].mode = "REF-RAW"
    #     self.color_sensors[1].mode = "REF-RAW"
    #
    #     while True:
    #         color_data = self.get_sensor_data("ColorSensor", "REF-RAW")
    #         left = color_data[0]
    #
    #         if self.within_range(actual=left, expected=white_value, amplitude=max_white_var):
    #             self.motors.left.run_forever(speed_sp=default_speed - speed_to_get_on_black_line)
    #             self.motors.right.run_forever(speed_sp=default_speed)
    #             continue
    #
    #         l_control = l_pid(left)
    #
    #         l_speed = default_speed + l_control
    #
    #         if l_speed > max_value:
    #             l_speed = max_value
    #         elif l_speed < min_value:
    #             l_speed = min_value
    #
    #         self.motors.left.run_forever(speed_sp=default_speed - l_speed)
    #         self.motors.right.run_forever(speed_sp=default_speed + l_speed)
    #
    #         # if color_data[0] > 500 or color_data[1] > 500:
    #         #     self.motors.left.stop()
    #         #     self.motors.right.stop()
    #         #     return

    @staticmethod
    def within_range(actual, expected, amplitude, all_values_positive=True):
        if all_values_positive:
            dif = abs(expected - actual)

            if dif <= amplitude:
                return True
            return False

    def request_area(self, area):
        pass

    def request_pipe(self, size):
        pass

    def pipe_rescue(self, size):
        pass

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
            # print(lis)

            if left <= max_approach_k or right <= max_approach_k or upper_front <= max_approach_k:
                first_key = True
                if abs(left - right) > k_to_identify_perpendicular and upper_front < 15:
                    # print("its perpendicular")
                    break

            if first_key and abs(left - right) <= 1 and min(left, right) < 5:
                second_key = True

            if first_key and second_key:
                # print("its parallel")
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

    def color_alignment(self, color="White"):
        self.stop_motors()
        # print("trying to alignment with ", color, " at speed ", -200)

        color_data = self.get_sensor_data("ColorSensor")
        while not (color_data[0] == color and color_data[1] == color):
            color_data = self.get_sensor_data("ColorSensor")
            if color_data[0] == color and color_data[1] != color:
                while color_data[1] != color:
                    color_data = self.get_sensor_data("ColorSensor")
                    self.motors.right.run_forever(speed_sp=-200)
                self.motors.right.stop()

            if color_data[0] != color and color_data[1] == color:
                while color_data[0] != color:
                    color_data = self.get_sensor_data("ColorSensor")
                    self.motors.left.run_forever(speed_sp=-200)
                self.motors.left.stop()

            if color_data[0] == color_data[1] and color_data[1] != color:
                while color_data[0] != color:
                    color_data = self.get_sensor_data("ColorSensor")
                    self.motors.left.run_forever(speed_sp=-200)
                self.motors.left.stop()

                while color_data[1] != color:
                    color_data = self.get_sensor_data("ColorSensor")
                    self.motors.right.run_forever(speed_sp=-200)
                self.motors.left.stop()

    def pid_alignment(self):
        pass

    def gap_fill(self):
        pass

    def gap_measurement(self):
        pass

    def underground_position_reset(self, side=None):
        self.stop_motors()
        self.move_timed(how_long=0.5)

        if side == "left":
            color_data = self.get_sensor_data("ColorSensor")
            while "Blue" not in color_data or "Black" not in color_data:
                self.motors.right.run_forever(speed_sp=DEFAULT_SPEED)
                self.motors.left.run_forever(speed_sp=DEFAULT_SPEED)
                color_data = self.get_sensor_data("ColorSensor")

            print("Fim de while com blue || black")
            self.stop_motors()
            self.move_timed(how_long=0.8, direction="forward", speed=DEFAULT_SPEED)
            return

        else:
            color_data = self.get_sensor_data("ColorSensor")
            self.move_timed(how_long=0.8, direction="forward", speed=DEFAULT_SPEED)
            while "Blue" not in color_data or "Black" not in color_data:
                print("color_data = ", color_data)
                self.motors.right.run_forever(speed_sp=DEFAULT_SPEED)
                self.motors.left.run_forever(speed_sp=DEFAULT_SPEED)
                color_data = self.get_sensor_data("ColorSensor")

            self.color_alignment("Black")
            self.stop_motors()
            self.move_timed(how_long=0.8, direction="forward", speed=DEFAULT_SPEED)

            self.rotate(angle=-80)
            upper_dist = self.get_sensor_data("InfraredSensor")[2]

            default_speed = 500
            pid = PID(12, 0, 10, setpoint=15)
            counter = 0

            while counter < 10:
                upper_dist = self.get_sensor_data("InfraredSensor")[2]
                side_distance = self.get_sensor_data("InfraredSensor")[0]
                front_distance = self.get_sensor_data("InfraredSensor")[1]
                control = pid(side_distance)

                #print(side_distance, front_distance)

                if upper_dist >= 60:
                    counter += 1

                speed_a = default_speed + control
                speed_b = default_speed - control

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

            self.stop_motors()
        print("Fim underground")

    def avoid_collision(self):
        if self.get_sensor_data("InfraredSensor")[1] < 40:
            possible_options = ["left", "right "]

            color_data = self.get_sensor_data("ColorSensor")
            while True:
                color_data = self.get_sensor_data("ColorSensor")
                self.motors.left.run_forever(speed_sp=DEFAULT_SPEED)
                self.motors.right.run_forever(speed_sp=DEFAULT_SPEED)

                if "Undefined" in color_data:
                    pass