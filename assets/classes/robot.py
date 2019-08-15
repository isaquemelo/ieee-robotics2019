#!/usr/bin/env python3
from .duo import Duo
import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
import json

DEFAULT_SPEED = 400


def map_values(n, start1, stop1, start2, stop2):
    return ((n - start1) / (stop1 - start1)) * (stop2 - start2) + start2


class Robot:
    ev3.Sound.speak("Robot started...")

    def __init__(self):
        self.DEFAULT_SPEED = 400

        # define sensors
        self.gyroscope_sensor = ev3.GyroSensor('in3')
        self.gyroscope_sensor.mode = 'GYRO-RATE'
        self.gyroscope_sensor.mode = 'GYRO-ANG'

        self.color_sensors = Duo(ev3.ColorSensor('in1'), ev3.ColorSensor('in2'))
        self.ultrasonic_sensor = ev3.UltrasonicSensor('in4')
        self.infrared_sensors = (0, 0)

        # define motors
        self.motors = Duo(ev3.LargeMotor('outA'), ev3.LargeMotor('outD'), ev3.LargeMotor('outC'))
        # self.handler = ev3.LargeMotor('outC')

        # define status
        self.historic = [""]

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

    def rotate(self, angle, axis="own", speed=DEFAULT_SPEED):

        if 30 > angle > 0:
            speed = map_values(math.fabs(angle), 0, 90, 100, 1000)

        reverse = False

        if angle < 0:
            reverse = True
            angle = angle * -1

        self.gyroscope_sensor.mode = 'GYRO-RATE'
        self.gyroscope_sensor.mode = 'GYRO-ANG'

        start_angle = self.get_sensor_data('GyroSensor')
        now_angle = start_angle

        self.motors.left.stop()
        self.motors.right.stop()

        while now_angle < angle + start_angle:
            if reverse:
                if axis == "own":
                    self.motors.left.run_forever(speed_sp=-speed)
                    self.motors.right.run_forever(speed_sp=speed)
                else:
                    self.motors.right.run_forever(speed_sp=speed)

                now_angle = self.get_sensor_data('GyroSensor') * -1
            else:
                if axis == "own":
                    self.motors.left.run_forever(speed_sp=speed)
                    self.motors.right.run_forever(speed_sp=-speed)
                else:
                    self.motors.left.run_forever(speed_sp=speed)
                now_angle = self.get_sensor_data('GyroSensor')

        self.motors.left.stop()
        self.motors.right.stop()

        self.gyroscope_sensor.mode = 'GYRO-RATE'
        self.gyroscope_sensor.mode = 'GYRO-ANG'

    def move_timed(self, how_long=0.3, direction="forward", speed=DEFAULT_SPEED):
        end_time = datetime.now() + timedelta(seconds=how_long)

        vel = speed

        if direction != "forward":
            vel = -speed

        while datetime.now() < end_time:
            self.motors.left.run_forever(speed_sp=vel), self.motors.right.run_forever(speed_sp=vel)
        self.motors.left.stop(), self.motors.right.stop()

    def run_action(self, direction, still_learning=True):
        print("Run action chamada com os seguintes paramentros:", "direction:", direction, "still_learning:",
              still_learning)
        # print("CHAMOU O RUN_ACTION COM O REVERSE_PATH = {}".format(self.reverse_path))
        self.realigment_counter = 0
        # if self.nao_pode:
        #     self.move_timed(how_long=0.2, direction="back")
        #     return
        # print("self.primeiro_bounding_box = {}".format(self.primeiro_bounding_box))
        # print("self.learned_colors_is_empity() = {}".format(self.learned_colors_is_empity()))
        # if self.primeiro_bounding_box is False and self.learned_colors_is_empity():
        #     print("TAVA NO INICIO DA PISTA")
        #     self.tempo_para_chamar_run_action = datetime.now() + timedelta(seconds=5)
        #     return

        if datetime.now() < self.tempo_para_chamar_run_action:
            # print("CHAMOU A RUN_ACTION NO TEMPO ERRADO")
            ev3.Sound.beep()
            ev3.Sound.beep()
            self.move_timed(how_long=0.2, direction="back")
            return
        else:
            # print("CHAMOU A RUN_ACTION NO TEMPO CERTO")
            self.tempo_para_chamar_run_action = datetime.now() + timedelta(seconds=5)

            if self.rect_color in self.learned_colors.keys():
                if self.reverse_path is True:
                    self.learned_colors[self.rect_color][-1] -= 1
                else:
                    self.learned_colors[self.rect_color][-1] += 1
                # print("EXECUTOU O DESCARREGAMENTO DA COR")
                print(self.learned_colors)
                self.stop_motors()
                # if self.ta_no_final_da_pista is True:
                #     self.ta_no_final_da_pista = False
                #     self.rotate(180)

                # if self.primeiro_bounding_box is False:
                # if self.voltou is False:
                if self.learned_colors_is_empity() is True:
                    if self.reverse_path is True:
                        for i in range(5):
                            ev3.Sound.beep()
                        self.ta_no_final_da_pista = True
                        self.rotate(180)
                        self.reverse_path = False
                        for k in sorted(self.learned_colors.keys()):
                            if k == self.rect_color:
                                self.learned_colors[k][-1] = 1
                            else:
                                self.learned_colors[k][-1] = 0
                        # self.tempo_para_chamar_run_action = datetime.now() + timedelta(seconds=5)
                        return
                    # else:
                    #     self.ta_na_ranpa = True
                    #     for i in range(7):
                    #         print("TA NA RAMPA")
                if self.learned_colors_is_full() is True:
                    self.ta_na_ranpa = True
                    for i in range(7):
                        print("TA NA RAMPA")

            self.nao_pode = True

            # if self.reverse_path:
            #     if not still_learning:
            #         if direction == "forward":
            #             pass
            #         elif direction == "left":
            #             self.rotate(90, axis="own")
            #         elif direction == "right":
            #             self.rotate(-90, axis="own")
            #     else:
            #         self.rotate(-90, axis="own")
            #     # return None
            #
            # else:
            #     if not still_learning:
            #         if direction == "forward":
            #             pass
            #         elif direction == "left":
            #             self.rotate(-90, axis="own")
            #         elif direction == "right":
            #             self.rotate(90, axis="own")
            #     else:
            #         if :
            #             self.rotate(90, axis="own")
            #         else:
            #             self.rotate(90, axis="own")
            #     # return None

            if not still_learning:
                if direction == "forward":
                    pass
                elif direction == "left":
                    self.rotate(90 if self.reverse_path else -90, axis="own")
                elif direction == "right":
                    self.rotate(-90 if self.reverse_path else 90, axis="own")
            else:
                if self.has_came_from_json:
                    if direction == "forward":
                        pass
                    elif direction == "left":
                        self.rotate(-90, axis="own")
                    elif direction == "right":
                        self.rotate(90, axis="own")

                    self.has_came_from_json = False

                else:
                    self.rotate(90, axis="own")

        # return None

    def stop_motors(self):
        self.motors.left.stop()
        self.motors.right.stop()
