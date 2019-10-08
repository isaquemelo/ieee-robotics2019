#!/usr/bin/env python3
from assets.classes.duo import Duo
import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
from simple_pid import PID
# import json
import time
from time import sleep

import paho.mqtt.client as mqtt
from struct import *

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
        self.color_sensors[0].mode = "COL-COLOR"
        self.color_sensors[1].mode = "COL-COLOR"
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

        self.ultrasonic_sensors = {"left": 2550, "diagonal_top": ev3.UltrasonicSensor('in3')}
        self.infrared_sensors = {"right": 50, "left": 50, "front": 100}

        # define motors
        self.motors = Duo(ev3.LargeMotor('outB'), ev3.LargeMotor('outD'))
        self.motors.left.polarity = "inversed"
        self.motors.right.polarity = "inversed"

        self.handler = Duo(ev3.LargeMotor('outC'), ev3.LargeMotor('outC'))

        self.move_handler(3, direction="top", speed=1000)
        self.handler.left.run_forever(speed_sp=-1000)

        # define status
        self.historic = [""]

        # watter server settings
        self.has_pipe = False
        self.current_pipe_size = 10  # [10, 15, 20]

        self.status_dictionary = {"initialPositionReset": 0, "doneInitialPositionReset": 1, "rescuedPipe": 2,
                                  "placingPipe": 3, "donePlacingPipe": 4, "waiting": 5, "want10pipe": 6, "want20pipe": 7, "want15pipe": 8}
        self.robot_dictionary = {0: "waitingForWatterRobotToAlign", 1: "initialPositionReset", 2: "pipeRescuing",
                                 3: "pipeInPositionToRescue-10", 4: "pipeInPositionToRescue-20", 5: "pipeInPositionToRescue-15"}

        self.status = self.status_dictionary["initialPositionReset"]
        self.robot_status = 0

        self.secondary_brick_ip = "101.42.0.36"
        self.client = mqtt.Client()
        self.client.connect(self.secondary_brick_ip, 1883, 60)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.loop_start()

        # sever settings
        self.local_ip = "localhost"
        self.receiver = mqtt.Client()
        self.receiver.connect(self.local_ip, 1883, 60)
        self.receiver.on_connect = self.receiver_on_client_connect
        self.receiver.on_message = self.receiver_on_client_message
        self.receiver.loop_start()

        # self.external_ip = "192.168.0.1"
        # self.publisher = mqtt.Client()
        # self.publisher.connect(self.external_ip, 1883, 60)
        # self.publisher.on_connect = self.publisher_on_client_connect
        # # self.publisher.on_message = self.publisher_on_client_message
        # self.publisher.on_publish = self.publisher_on_client_publish
        # self.publisher.loop_start()

    def publisher_on_client_publish(self, client, userdata, result):  # create function for callback
        # print("data published")
        pass

    def publisher_on_client_message(self, client, userdata, message):
        print("message received: ", end="")
        payload = unpack("i", message.payload)
        print(payload[0])

    def publisher_on_client_connect(self, client, userdata, flags, rc):
        print("The bluetooth bricks are connected with result code", str(rc))
        client.subscribe("topic/status")

    def receiver_on_client_message(self, client, userdata, message):
        print("message received: ", end="")
        payload = unpack("i", message.payload)
        self.robot_status = self.robot_dictionary[int(payload[0])]
        # self.ultrasonic_sensors['left'] = payload[0]
        # self.ultrasonic_sensors['right'] = payload[1]
        # self.infrared_sensors['front'] = payload[2]
        # self.infrared_sensors['left'] = payload[3]

        print(payload)

    def receiver_on_client_connect(self, client, userdata, flags, rc):
        print("The bluetooth bricks are connected with result code", str(rc))
        client.subscribe("topic/PipeLineRobot")

    def publish_data(self):
        message = pack("i", self.status)
        self.publisher.publish("topic/status", message, qos=0)
        print("dado publicado:", unpack("i", message))

    def on_message(self, client, userdata, message):
        # print("info received")
        payload = unpack("iiiid", message.payload)
        self.infrared_sensors['front'] = payload[0]
        self.ultrasonic_sensors['left'] = payload[1]
        self.infrared_sensors['left'] = payload[2]
        self.infrared_sensors['right'] = payload[3]
        # print(payload)

    def on_connect(self, client, userdata, flags, rc):
        print("The robots are connected with result code", str(rc))
        client.subscribe("topic/sensors")

    def reset_gyroscope(self):
        self.gyroscope_sensor.mode = 'GYRO-RATE'
        self.gyroscope_sensor.mode = 'GYRO-ANG'

    def get_sensor_data(self, sensor_name, ColorSensorMode="COL-COLOR"):
        # returns the value of a sensor

        if sensor_name == "InfraredSensor":
            # return [self.infrared_sensors["left"], self.infrared_sensors["right"],
            #         self.infrared_sensors["diagonal_top"].value()]
            return [self.infrared_sensors["left"], self.infrared_sensors["right"], self.infrared_sensors["front"]]

        elif sensor_name == "GyroSensor":
            return self.gyroscope_sensor.angle

        elif sensor_name == "Ultrasonic":
            # return self.ultrasonic_sensor.value() / 10
            # return [self.ultrasonic_sensors['left'] / 10, self.ultrasonic_sensors['bottom'] / 10]
            return [self.ultrasonic_sensors['left'] / 10, self.ultrasonic_sensors["diagonal_top"].value() / 10]

        elif sensor_name == "ColorSensor":
            if ColorSensorMode != "COL-COLOR":
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

    def move_handler(self, how_long=7, direction="down", speed=50):
        end_time = datetime.now() + timedelta(seconds=how_long)

        vel = speed

        if direction != "down":
            vel = -speed

        while datetime.now() < end_time:
            self.handler.left.run_forever(speed_sp=vel)
        self.handler.left.stop()

    def pipeline_support_following(self):
        self.stop_motors()
        print("pipeline_support_following")
        default_speed = 150

        pid = PID(12, 0, 6, setpoint=2)
        front_distance_to_rotate = 2

        speed_a = 0
        speed_b = 0

        border_situation = False
        initial_time = datetime.now()
        time = 0

        last_pipe_distance = self.get_sensor_data("Ultrasonic")[0]
        last_last_pipe_distance = 255

        self.color_sensors[0].mode = "COL-REFLECT"
        self.color_sensors[1].mode = "COL-REFLECT"
        while True:
            side_distance = self.get_sensor_data("InfraredSensor")[0]
            front_distance = self.get_sensor_data("InfraredSensor")[2]
            pipe_distance = self.get_sensor_data("Ultrasonic")[0]
            upper_distance = self.get_sensor_data("Ultrasonic")[1]
            control = pid(side_distance)

            # print(side_distance, front_distance)
            # print("control = ", control)

            color_data = self.get_sensor_data("ColorSensor", "r")
            if color_data[0] == 0 or color_data[1] == 0:
                self.stop_motors()
                if self.get_sensor_data("Ultrasonic")[1] > 23:
                    print("finished pipeline_support_following 'cause found the edge upper_dist =", upper_distance)
                    return

            if pipe_distance > 40 and abs(pipe_distance - last_pipe_distance) < 10:
                self.align_with_hole()
                has_pipe_already = self.has_pipe_check()

                if has_pipe_already is False:  # and robot is with pipe
                    print("Hole detect! Placing pipe ")
                    # self.move_handler(10, speed=10)
                    self.move_timed(0.5, direction="backwards", speed=300)
                    self.move_handler(1, direction="top", speed=1000)
                    self.handler.left.run_forever(speed_sp=-150)
                    self.rotate(90, speed=90)

                elif has_pipe_already is True:
                    print("Already has pipe! Misguided sensor info")
                    self.rotate(90, speed=90)
                    continue
                elif has_pipe_already is None:
                    print("Info not reliable at all!")
                    self.rotate(90, speed=90)
                    continue

            if front_distance < front_distance_to_rotate:
                self.stop_motors()
                print("rotating cause it found small front dist")
                if border_situation is not None:
                    if border_situation is False:
                        # print("cruva sem risco")
                        # sleep(3)
                        self.move_timed(how_long=0.2, direction="backwards")
                        self.rotate(0, axis="own", speed=90)
                        time = datetime.now()
                        border_situation = True
                    elif border_situation and not (datetime.now() - initial_time > timedelta(seconds=3)):
                        border_situation = None
                        # print("curva com provavel risco")
                        if datetime.now() - time <= timedelta(seconds=1.7):
                            # print("realmente havia risco")
                            self.rotate(90, axis="own", speed=90)
                        else:
                            # print("na verdade nao havia risco")
                            self.move_timed(how_long=0.2, direction="backwards")
                            self.rotate(90, axis="own", speed=90)
                        # sleep(3)
                    elif border_situation and (datetime.now() - initial_time > timedelta(seconds=3)):
                        border_situation = None
                        # print("curva sem risco")
                        # sleep(3)
                        self.move_timed(how_long=0.2, direction="backwards")
                        self.rotate(90, axis="own", speed=90)
                elif border_situation is None:
                    # print("curva sem risco")
                    # sleep(3)
                    self.move_timed(how_long=0.2, direction="backwards")
                    self.rotate(90, axis="own", speed=90)

            speed_a = default_speed + control
            speed_b = default_speed - control

            # print(control)

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

            last_last_pipe_distance = last_pipe_distance
            last_pipe_distance = pipe_distance

    def pipe_10(self):
        k = 6.7
        inverse = -1
        self.handler.left.stop_action = "hold"
        self.handler.left.stop()
        while True:
            self.move_handler(how_long=0.2, direction="down", speed=50)
            self.handler.left.stop()
            for i in range(3):
                self.rotate(3 if inverse == -1 else 6, speed=90)
                self.rotate(-3 if inverse == 1 else -6, speed=90)
                self.move_handler(how_long=0.2, direction="up", speed=50)
                self.move_handler(how_long=0.1, direction="down", speed=50)
                inverse *= -1
        ev3.Sound.beep()

    def place_pipe(self):
        inverse = -1
        self.move_handler(how_long=2.2, speed=20)
        self.handler.left.stop_action = "hold"
        self.handler.left.stop()
        while True:
            self.rotate(3 if inverse == -1 else 6, speed=90)
            self.move_timed(0.1, speed=300)
            self.rotate(-3 if inverse == 1 else -6, speed=90)
            self.move_timed(0.1, speed=300)

            inverse *= -1

    def has_pipe_check(self) -> bool:
        top_values = []

        pipe_distance = self.get_sensor_data("Ultrasonic")[0]
        upper_distance = self.get_sensor_data("Ultrasonic")[1]
        self.move_timed(2, speed=250)
        top_values.append(self.get_sensor_data("Ultrasonic")[1])
        self.move_timed(1, direction="backwards", speed=100)
        top_values.append(self.get_sensor_data("Ultrasonic")[1])
        self.move_timed(0.7, direction="backwards", speed=100)
        top_values.append(self.get_sensor_data("Ultrasonic")[1])

        print(top_values)
        if top_values[0] < top_values[2] and top_values[1] < top_values[2]:
            if top_values[0] <= 9:
                return True
            elif top_values[0] >= 11:
                return False
            else:
                return None

    def align_with_hole(self) -> int:
        self.stop_motors()
        default_speed = 150
        self.color_sensors[0].mode = "COL-REFLECT"
        self.color_sensors[1].mode = "COL-REFLECT"

        pid = PID(12, 0, 6, setpoint=2)

        speed_a = 0
        speed_b = 0

        initial_time = datetime.now()

        while True:
            side_distance = self.get_sensor_data("InfraredSensor")[0]
            front_distance = self.get_sensor_data("InfraredSensor")[2]
            pipe_distance = self.get_sensor_data("Ultrasonic")[0]
            upper_distance = self.get_sensor_data("Ultrasonic")[1]
            control = pid(side_distance)

            color_data = self.get_sensor_data("ColorSensor", "r")
            if color_data[0] == 0 or color_data[1] == 0:
                self.stop_motors()
                if self.get_sensor_data("Ultrasonic")[1] > 23:
                    ev3.Sound.beep()
                    print("found end of the pipeline")
                    return

            # print(side_distance, front_distance)
            # print("control = ", control)

            if pipe_distance < 20:
                # self.move_timed(0.3, speed=300)
                self.stop_motors()
                end_hole_time = datetime.now()
                delta_time = (end_hole_time - initial_time)/2
                print(delta_time.seconds + delta_time.microseconds/10**6)
                self.move_timed(delta_time.seconds + delta_time.microseconds/10**6, direction="backwards", speed=150)
                self.rotate(-90, speed=100)
                return

            if datetime.now() == initial_time + timedelta(seconds=1.5):
                print("Alignment took to long, canceling...")
                end_hole_time = datetime.now()
                # delta_time = (end_hole_time - initial_time)
                # self.move_timed(delta_time.seconds + delta_time.microseconds / 10 ** 6, direction="backwards",
                #                 speed=150)
                # break

            speed_a = default_speed + control
            speed_b = default_speed - control

            # print(control)

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

    def adjust_corner_to_go_green(self):
        self.rotate(angle=-80)
        self.move_timed(how_long=1, direction="backward")
        self.rotate(angle=80)
        self.rotate(angle=80)
        return

    def go_to_green(self):
        self.color_sensors[0].mode = "COL-REFLECT"
        self.color_sensors[1].mode = "COL-REFLECT"

        bord_dist_expected = 30
        default_speed = 400
        max_value = 800
        min_value = max_value * -1
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

    @staticmethod
    def within_range(actual, expected, amplitude, all_values_positive=True):
        if all_values_positive:
            dif = abs(expected - actual)

            if dif <= amplitude:
                return True
            return False

    def pipe_rescue(self, size=0):
        default_speed = 300
        reduce_speed = 150

        while True:
            self.motors.left.run_forever(speed_sp=default_speed)
            self.motors.right.run_forever(speed_sp=default_speed)
            color_data = self.get_sensor_data("ColorSensor")

            print(color_data)

            if color_data[1] == "Undefined":
                while self.get_sensor_data("ColorSensor")[1] != "White":
                    self.motors.left.run_forever(speed_sp=default_speed - reduce_speed)
                    self.motors.right.run_forever(speed_sp=default_speed)

                self.motors.left.stop()
                self.motors.right.stop()
                continue

            if color_data[1] == "White":
                while self.get_sensor_data("ColorSensor")[1] != "Undefined":
                    self.motors.left.run_forever(speed_sp=default_speed)
                    self.motors.right.run_forever(speed_sp=default_speed - reduce_speed)

                self.motors.left.stop()
                self.motors.right.stop()
                continue

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

    def underground_position_reset(self):
        self.stop_motors()
        self.move_timed(how_long=0.5)

        # while self.get_sensor_data("")

        self.color_sensors[0].mode = "REF-RAW"
        self.color_sensors[1].mode = "REF-RAW"

        while self.color_sensors[0].value() < 600:
            self.motors.right.run_forever(speed_sp=800)
            self.motors.left.run_forever(speed_sp=800)

        self.stop_motors()
        ev3.Sound.beep().wait()
        self.move_timed(0.7, direction="forward")

        self.stop_motors()

        self.color_sensors[0].mode = "COL-REFLECT"
        self.color_sensors[1].mode = "COL-REFLECT"

        self.stop_motors()
        print("Fim de while com blue || black")
        self.rotate(-80, speed=150)  # talvez isso deveria ser -80 e não 80

        default_speed = 500
        pid = PID(12, 0, 10, setpoint=15)
        counter = 0

        while True:
            upper_dist = self.get_sensor_data("InfraredSensor")[2]
            side_distance = self.get_sensor_data("InfraredSensor")[0]
            control = pid(side_distance)
            color_data = self.get_sensor_data("ColorSensor")

            print("diagonal sensors", upper_dist)

            if upper_dist >= 55:
                counter += 1
            else:
                counter = 0

            if counter > 1 or "Undefined" in color_data:
                self.stop_motors()
                self.move_timed(how_long=0.5, direction="backwards", speed=1000)
                break

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

            print(speed_a, speed_b)
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

    def avoid_falling(self, direction):
        self.stop_motors()
        print("avoid falling off")
        k_rotation = 20
        left_dist = self.get_sensor_data()
        right_dist = self.get_sensor_data()

        self.move_timed(how_long=0.3, direction=direction)

        if left_dist > 30:
            self.rotate(angle=k_rotation)
            return
        if right_dist > 30:
            self.rotate(angle=-k_rotation)
            return

        return

    def sensors_verification(self):
        self.stop_motors()
        self.handler.left.stop()

        while True:
            infrared = self.get_sensor_data("InfraredSensor")
            ultrasom = self.get_sensor_data("Ultrasonic")
            gyro = self.get_sensor_data("GyroSensor")
            color = self.get_sensor_data("ColorSensor")
            print(infrared)
            print(ultrasom)
            print(color)
            print(gyro)
            time.sleep(2)

    def black_line_following(self, side):
        self.color_sensors[0].mode = "COL-REFLECT"
        self.color_sensors[1].mode = "COL-REFLECT"
        found_pipe = False
        begin = None
        k_found_pipe = 25

        pid = PID(1.025, 0, 0.2, setpoint=39)
        default = 300
        max_speed_bound = 500
        max_control = max_speed_bound - default
        min_control = max_speed_bound + default
        first_time = True

        if side == "left":
            side = 0
        else:
            side = 1

        while True:
            control = pid(int(self.get_sensor_data("ColorSensor", "r")[side]))
            print(default - control, default + control)

            bottom_front = self.get_sensor_data("InfraredSensor")[2]
            if bottom_front <= k_found_pipe and first_time:
                found_pipe = True
                first_time = False
                begin = datetime.now() + timedelta(seconds=2)

                self.stop_motors()
                self.handler.left.run_forever(speed_sp=500)

            if found_pipe and datetime.now() >= begin:
                found_pipe = False
                self.handler.left.run_forever(speed_sp=-1000)

                c = 0
                value = 10 if side == "right" else -10

                while c < 5:
                    self.rotate(value, speed=200)
                    self.move_timed(0.3, speed=200)
                    c += 1

                color_data = self.get_sensor_data("ColorSensor", "r")

                while True:
                    color_data = self.get_sensor_data("ColorSensor", "r")

                    if color_data[0] < 50 or color_data[1] < 50:
                        self.color_alignment(True)
                        self.green_slope()
                        self.slope_following()
                        self.rotate(80, speed=150)
                        return
                        break

                    self.motors.left.run_forever(speed_sp=DEFAULT_SPEED)
                    self.motors.right.run_forever(speed_sp=DEFAULT_SPEED)

                break

            speed_a = default - control
            speed_b = default + control

            if speed_a >= max_speed_bound:
                speed_a = max_speed_bound

            elif speed_a <= -max_speed_bound:
                speed_a = -max_speed_bound

            if speed_b >= max_speed_bound:
                speed_b = max_speed_bound
            elif speed_b <= -max_speed_bound:
                speed_b = -max_speed_bound

            if side == 1:
                self.motors.left.run_forever(speed_sp=speed_a)
                self.motors.right.run_forever(speed_sp=speed_b)
            else:
                self.motors.left.run_forever(speed_sp=speed_b)
                self.motors.right.run_forever(speed_sp=speed_a)

        self.color_sensors[0].mode = "COL-COLOR"
        self.color_sensors[1].mode = "COL-COLOR"

        return

    def color_alignment(self, aligment_with_color=False, default_speed=200):
        self.stop_motors()
        print("called alignment_for_meeting_area_initial_setting")
        self.color_sensors[0].mode = "COL-REFLECT"
        self.color_sensors[1].mode = "COL-REFLECT"
        k_min_white_reflect = 50
        color_data = self.get_sensor_data("ColorSensor", "r")
        k_time = timedelta(0.7)
        k_rotation = 90

        while color_data[0] < k_min_white_reflect or color_data[1] < k_min_white_reflect:

            left_back_begin = datetime.now()
            while color_data[0] < k_min_white_reflect:
                color_data = self.get_sensor_data("ColorSensor", "r")
                self.motors.left.run_forever(speed_sp=-default_speed)
            self.stop_motors()
            left_back_end = datetime.now()

            right_back_begin = datetime.now()
            while color_data[1] < k_min_white_reflect:
                color_data = self.get_sensor_data("ColorSensor", "r")
                self.motors.right.run_forever(speed_sp=-default_speed)
            self.stop_motors()
            right_back_end = datetime.now()

            self.reset_gyroscope()
            left_front_begin = datetime.now()
            while color_data[0] >= k_min_white_reflect:
                if self.get_sensor_data("GyroSensor") > k_rotation:
                    self.stop_motors()
                    print("alignment failed")
                    self.move_timed(how_long=0.3, direction="backward")
                    return
                color_data = self.get_sensor_data("ColorSensor", "r")
                self.motors.left.run_forever(speed_sp=default_speed)
            self.stop_motors()
            left_front_end = datetime.now()

            self.reset_gyroscope()
            right_front_begin = datetime.now()
            while color_data[1] >= k_min_white_reflect:
                if self.get_sensor_data("GyroSensor") < -k_rotation:
                    self.stop_motors()
                    print("alignment failed")
                    self.move_timed(how_long=0.3, direction="backward")
                    return
                color_data = self.get_sensor_data("ColorSensor", "r")
                self.motors.right.run_forever(speed_sp=default_speed)
            self.stop_motors()
            right_front_end = datetime.now()

            max_time = max(
                [left_back_end - left_back_begin, right_back_end - right_back_begin, left_front_end - left_front_begin,
                 right_front_end - right_front_begin])

            if max_time <= k_time:
                while color_data[0] < k_min_white_reflect or color_data[1] < k_min_white_reflect:
                    color_data = self.get_sensor_data("ColorSensor", "r")
                    self.motors.left.run_forever(speed_sp=-default_speed)
                    self.motors.right.run_forever(speed_sp=-default_speed)
                self.stop_motors()
                break

        self.stop_motors()
        if aligment_with_color:
            color_data = self.get_sensor_data("ColorSensor", "r")
            end_time = datetime.now() + timedelta(seconds=1)
            while (color_data[0] >= k_min_white_reflect or color_data[1] >= k_min_white_reflect) and (
                    end_time >= datetime.now()):
                self.motors.left.run_forever(speed_sp=default_speed)
                self.motors.right.run_forever(speed_sp=default_speed)
                color_data = self.get_sensor_data("ColorSensor", "r")
        self.stop_motors()
        return

    def initial_location_reset(self):
        self.color_sensors[0].mode = "COL-REFLECT"
        self.color_sensors[1].mode = "COL-REFLECT"
        default_speed = 300
        expected_save_side_dist = 30
        k_min_white_reflect = 50
        k_dist_from_robot = 20

        while True:
            bottom_front_dist = self.get_sensor_data("InfraredSensor")[2]
            color_data = self.get_sensor_data("ColorSensor", "r")

            if color_data[0] < k_min_white_reflect or color_data[1] < k_min_white_reflect:
                self.stop_motors()
                if color_data[0] < k_min_white_reflect or color_data[1] < k_min_white_reflect:  # double check
                    ev3.Sound.beep()
                    print("found something different from white")
                    needs_to_be_on_color = self.get_out_of_color()
                    self.color_alignment(aligment_with_color=needs_to_be_on_color)
                    its_green_slope = self.verify_green_slope()
                    if its_green_slope:
                        print("found green slope")
                        ev3.Sound.beep()
                        ev3.Sound.beep()
                        ev3.Sound.beep()
                        self.green_slope()
                        self.slope_following()
                        break
                    else:
                        pass
                    self.move_timed(how_long=0.4, direction="backward")
                    left_dist = self.get_sensor_data("InfraredSensor")[0]
                    right_dist = self.get_sensor_data("InfraredSensor")[1]
                    if left_dist > expected_save_side_dist:
                        self.get_out_of_risk_edge_situation(side="left")
                    elif right_dist > expected_save_side_dist:
                        self.get_out_of_risk_edge_situation(side="right")
                    else:
                        print("rotation decision is save")
                        self.rotate(80)

            if bottom_front_dist <= k_dist_from_robot:
                self.stop_motors()
                print("found robot")
                ev3.Sound.beep()
                self.rotate(80)

            self.motors.left.run_forever(speed_sp=default_speed)
            self.motors.right.run_forever(speed_sp=default_speed)

        self.color_sensors[0].mode = "COL-COLOR"
        self.color_sensors[1].mode = "COL-COLOR"
        return

    def verify_green_slope(self):
        self.stop_motors()
        self.color_sensors[0].mode = "COL-COLOR"
        self.color_sensors[1].mode = "COL-COLOR"
        print("called verify_green_slope")
        color_data = self.get_sensor_data("ColorSensor")
        if (color_data[0] == "Green" and color_data[1] != "Undefined") or (
                color_data[1] == "Green" and color_data[0] != "Undefined"):
            self.color_sensors[0].mode = "COL-REFLECT"
            self.color_sensors[1].mode = "COL-REFLECT"
            return True
        self.color_sensors[0].mode = "COL-REFLECT"
        self.color_sensors[1].mode = "COL-REFLECT"
        return False

    def green_slope(self):
        self.stop_motors()
        print("called go_down_green_slope")
        default_speed = 400
        k_time = 3.5

        pid_side = self.adjust_before_go_down_green_slope()
        max_speed = 500
        min_speed = -max_speed
        if pid_side == "right":
            expected_save_side_dist = 35
        else:
            expected_save_side_dist = 30
        pid = PID(5, 0, 5, setpoint=expected_save_side_dist)

        if pid_side == "left":
            minimum_time_before_stop = datetime.now() + timedelta(seconds=k_time)
            while True:
                if datetime.now() > minimum_time_before_stop:
                    break

                left_dist = self.get_sensor_data("InfraredSensor")[0]

                control = pid(left_dist)

                left_speed = default_speed - control
                right_speed = default_speed + control

                if left_speed > max_speed:
                    left_speed = max_speed
                elif left_speed < min_speed:
                    left_speed = min_speed

                if right_speed > max_speed:
                    right_speed = max_speed
                elif right_speed < min_speed:
                    right_speed = min_speed

                self.motors.left.run_forever(speed_sp=left_speed)
                self.motors.right.run_forever(speed_sp=right_speed)

        elif pid_side == "right":
            minimum_time_before_stop = datetime.now() + timedelta(seconds=k_time)
            while True:
                if datetime.now() > minimum_time_before_stop:
                    break

                right_dist = self.get_sensor_data("InfraredSensor")[1]

                control = pid(right_dist)

                left_speed = default_speed + control
                right_speed = default_speed - control

                if left_speed > max_speed:
                    left_speed = max_speed
                elif left_speed < min_speed:
                    left_speed = min_speed

                if right_speed > max_speed:
                    right_speed = max_speed
                elif right_speed < min_speed:
                    right_speed = min_speed

                self.motors.left.run_forever(speed_sp=left_speed)
                self.motors.right.run_forever(speed_sp=right_speed)

        elif pid_side is None:
            k_time = k_time - 0.5
            minimum_time_before_stop = datetime.now() + timedelta(seconds=k_time)
            while True:
                if datetime.now() > minimum_time_before_stop:
                    break

                self.motors.left.run_forever(speed_sp=default_speed)
                self.motors.right.run_forever(speed_sp=default_speed)

        self.stop_motors()

    def adjust_before_go_down_green_slope(self):
        print("called adjust_before_go_down_green_slope")
        self.color_sensors[0].mode = "COL-REFLECT"
        self.color_sensors[1].mode = "COL-REFLECT"
        default_speed = 300
        k_min_white_reflect = 50
        expected_save_side_dist = 40

        color_data = self.get_sensor_data("ColorSensor", "r")
        while color_data[0] > k_min_white_reflect and color_data[1] > k_min_white_reflect:
            color_data = self.get_sensor_data("ColorSensor", "r")
            self.motors.left.run_forever(speed_sp=default_speed)
            self.motors.right.run_forever(speed_sp=default_speed)
        self.stop_motors()

        self.color_alignment()
        self.move_timed(how_long=0.2, direction="backward")
        left_dist = self.get_sensor_data("InfraredSensor")[0]
        right_dist = self.get_sensor_data("InfraredSensor")[1]

        if left_dist >= expected_save_side_dist:
            print("pid with left side to make sure the does not fall")
            return "left"
        if right_dist >= expected_save_side_dist:
            print("pid with right side to make sure the does not fall")
            return "right"

        print("pid is not necessary")
        return None

    def slope_following(self):
        print("called slope following")
        self.color_sensors[0].mode = "COL-REFLECT"
        self.color_sensors[1].mode = "COL-REFLECT"

        self.stop_motors()
        self.rotate(-80, speed=500)  # talvez isso deveria ser -80 e não 80

        default_speed = 300
        pid = PID(12, 0, 10, setpoint=12)

        k_to_find_end_by_color = 0
        while True:
            left_distance = self.get_sensor_data("InfraredSensor")[0]
            control = pid(left_distance)
            color_data = self.get_sensor_data("ColorSensor", "r")

            if k_to_find_end_by_color == color_data[0] and k_to_find_end_by_color == color_data[1]:
                self.move_timed(how_long=0.5, direction="backwards", speed=1000)
                print("found undefined on both color_sensors")
                break

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
        ev3.Sound.beep().wait()

    def get_out_of_color(self):
        self.stop_motors()
        default_speed = 200
        print("called get_out_of_color")
        may_be_green_slope = True

        self.color_sensors[0].mode = "COL-COLOR"
        self.color_sensors[1].mode = "COL-COLOR"

        color_data = self.get_sensor_data("ColorSensor")
        if color_data[0] in ["Blue", "Brown", "Red", "Yellow"] or color_data[1] in ["Blue", "Brown", "Red", "Yellow"]:
            may_be_green_slope = False
            print("got into a the color = ", color_data)
            while color_data[0] != "White" or color_data[1] != "White":
                print(color_data)
                self.motors.left.run_forever(speed_sp=-default_speed)
                self.motors.right.run_forever(speed_sp=-default_speed)
                color_data = self.get_sensor_data("ColorSensor")

        self.stop_motors()
        self.color_sensors[0].mode = "COL-REFLECT"
        self.color_sensors[1].mode = "COL-REFLECT"
        return may_be_green_slope

    def climb_green_slope(self):
        self.stop_motors()
        print("called climb_green_slope")
        default_speed = 400
        counter = 0

        self.color_sensors[0].mode = "COL-COLOR"
        self.color_sensors[1].mode = "COL-COLOR"

        while True:
            color_data = self.get_sensor_data("ColorSensor")

            if color_data[0] == "White" and color_data[1] == "White":
                counter += 1
                print(color_data, "contou")
            else:
                counter = 0
                print(color_data, "zerou")

            if counter >= 10:
                self.stop_motors()
                break

            self.motors.left.run_forever(speed_sp=default_speed)
            self.motors.right.run_forever(speed_sp=default_speed)

    def get_on_position_before_black_line_flw(self, side):
        self.stop_motors()
        ev3.Sound.beep()
        print("called get_on_position_before_black_line_flw")
        k_min_white_reflect = 50
        default_speed = 100

        self.color_sensors[0].mode = "COL-REFLECT"
        self.color_sensors[1].mode = "COL-REFLECT"

        while True:
            color_data = self.get_sensor_data("ColorSensor", "r")

            if color_data[0] < k_min_white_reflect or color_data[1] < k_min_white_reflect:
                self.stop_motors()
                needs_to_be_on_color = self.get_out_of_color()
                if not needs_to_be_on_color:
                    self.move_timed(how_long=0.2, direction="backward")
                    if default_speed - 100 >= 100:
                        default_speed -= 100
                        pass
                    continue
                break

            self.motors.left.run_forever(speed_sp=default_speed)
            self.motors.right.run_forever(speed_sp=default_speed)

        self.color_alignment(aligment_with_color=True, default_speed=100)

        color_data = self.get_sensor_data("ColorSensor", "r")
        if side == "left":
            side = 0
        else:
            side = 1
        while color_data[side] >= k_min_white_reflect:
            self.motors.left.run_forever(speed_sp=100)
            self.motors.right.run_forever(speed_sp=100)
            color_data = self.get_sensor_data("ColorSensor", "r")
        self.stop_motors()

    def adjust_before_black_line_flw(self, side):
        self.stop_motors()
        print("called adjust_before_black_line_flw")
        default_speed = 200

        self.reset_gyroscope()
        gyro = self.get_sensor_data("GyroSensor")

        if side == "left":
            while gyro < 71:
                self.motors.right.run_forever(speed_sp=-default_speed)
                gyro = self.get_sensor_data("GyroSensor")
            self.stop_motors()

        else:
            while gyro > -71:
                self.motors.left.run_forever(speed_sp=-default_speed)
                gyro = self.get_sensor_data("GyroSensor")
            self.stop_motors()
        sleep(3)

        while True:
            color_date = self.get_sensor_data("ColorSensor", "r")

            if side == "left":
                self.motors.left.run_forever(speed_sp=200)
                self.motors.right.run_forever(speed_sp=200 + 20)
                if color_date[0] < 50:
                    self.stop_motors()
                    self.rotate(angle=3, speed=150)
                    break
            else:
                self.motors.left.run_forever(speed_sp=200 + 20)
                self.motors.right.run_forever(speed_sp=200)
                if color_date[1] < 50:
                    self.stop_motors()
                    self.rotate(angle=-3, speed=150)
                    break

    def go_grab_pipe_routine(self, side):
        self.stop_motors()
        print("called go_grab_pipe_routine")
        if side == "left":
            print("using left side")
            self.rotate(angle=80)
            self.move_timed(how_long=0.5, direction="forward")
            self.rotate(angle=-80)

        else:
            print("using right side")
            self.move_timed(how_long=0.5, direction="backward")
            self.rotate(angle=-80)

        self.climb_green_slope()
        self.get_on_position_before_black_line_flw(side)
        self.adjust_before_black_line_flw(side)
        self.black_line_following(side)

    def get_out_of_risk_edge_situation(self, side):
        self.stop_motors()
        print("get_out_of_risk_edge_situation, with side", side)
        default_speed = 200
        k_angle = 45
        self.reset_gyroscope()
        gyro = self.get_sensor_data("GyroSensor")

        if side == "left":
            while gyro > -k_angle:
                self.motors.left.run_forever(speed_sp=-default_speed)
                gyro = self.get_sensor_data("GyroSensor")

        else:
            while gyro < k_angle:
                self.motors.right.run_forever(speed_sp=-default_speed)
                gyro = self.get_sensor_data("GyroSensor")

        self.stop_motors()
        self.move_timed(how_long=0.3, direction="backward")
        self.rotate(angle=160)