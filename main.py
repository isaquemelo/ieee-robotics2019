#!/usr/bin/env python3

import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
import time
import paho.mqtt.client as mqtt
from struct import *
from simple_pid import PID
from assets.classes.robot import Robot
import json
from assets.handlers.button import ButtonApproach
from assets.handlers.undefined_dealing import undefined_dealing
from assets.handlers.server_config import Server

DEFAULT_SPEED = 350



# 15.6 0 4.8
#pid = PID(15.6, 0, 4.8, setpoint=-4)

robot = Robot()
server = Server()


# client = mqtt.Client()
# client.connect("192.168.137.3", 1883, 60)
# client.loop_start()
# client.on_connect = on_connect
# client.on_message = on_message


def on_message(client, userdata, message):
    #print("mensagem recebida")
    payload = unpack("iiid", message.payload)
    robot.color_sensors = payload[1:3]
    robot.ultrasonic_sensors['top-left'] = payload[0]
    #print(payload)


def on_connect(client, userdata, flags, rc):
    print("The robots are connected with result code", str(rc))
    client.subscribe("topic/sensors")


server.client.on_connect = on_connect
server.client.on_message = on_message

server.client.loop_start()


def main():
    try:
        #robot.pipe_rescue()
        while True:
            print(ev3.InfraredSensor('in2').value(), ev3.InfraredSensor('in3').value())
            robot.pipe_rescue()
            break
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