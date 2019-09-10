#!/usr/bin/env python3

import ev3dev.ev3 as ev3
# import math
# from datetime import datetime, timedelta
# import time
# import paho.mqtt.client as mqtt
# from struct import *
# from simple_pid import PID
from assets.classes.PipeLineRobot import PipeLineRobot
# import json
# from assets.handlers.button import ButtonApproach
# from assets.handlers.undefined_dealing import undefined_dealing
# from assets.handlers.server_config import Server

DEFAULT_SPEED = 350



# 15.6 0 4.8
#pid = PID(15.6, 0, 4.8, setpoint=-4)

robot = PipeLineRobot()
#server = Server()


# client = mqtt.Client()
# client.connect("192.168.137.3", 1883, 60)
# client.loop_start()
# client.on_connect = on_connect
# client.on_message = on_message


# def on_message(client, userdata, message):
#     #print("mensagem recebida")
#     payload = unpack("iiiid", message.payload)
#     robot.color_sensors = payload[1:3]
#     robot.infrared_sensors["upper_front"] = payload[-2]
#     robot.ultrasonic_sensors['top'] = payload[0]
#     #print(payload)
#
#
# def on_connect(client, userdata, flags, rc):
#     print("The robots are connected with result code", str(rc))
#     client.subscribe("topic/sensors")
#
#
# server.client.on_connect = on_connect
# server.client.on_message = on_message
#
# server.client.loop_start()

# 51 lateral
# 110 frente

def main():
    try:
        #robot.rotate(80, axis="own", speed=90)
        #robot.pipeline_support_following()
        ## robot.black_line_following()
        robot.black_line_following_ac()
        # while True:
        #     print(robot.get_sensor_data("ColorSensor"))
        #robot.pipe_rescue(15)

        #robot.pipeline_support_following()
        #robot.get_in_position_to_grab_pipe()
            #time.sleep(8)
            # break

            # robot.motors.left.run_forever(speed_sp=speed)
            # robot.motors.right.run_forever(speed_sp=speed)
            # colors = robot.get_sensor_data("ColorSensor")
            # print(colors)
            # if colors[0] != "Blue" and colors[1] != "Blue":
            #     robot.stop_motors()
            #     break

    except KeyboardInterrupt:
        robot.motors.right.stop()
        robot.motors.left.stop()
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
    robot.motors.alternative.stop()

    server.client.loop_stop()
    server.client.disconnect()