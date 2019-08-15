#!/usr/bin/env python3

import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
import time
import paho.mqtt.client as mqtt
from struct import *
from simple_pid import PID
from assets.classes.robot import Robot
from rescue import rescue, bounding_box
import json
from assets.handlers.button import ButtonApproach
from assets.handlers.undefined_dealing import undefined_dealing
from assets.handlers.server_config import Server

DEFAULT_SPEED = 350



# 15.6 0 4.8
pid = PID(15.6, 0, 4.8, setpoint=-4)

robot = Robot()
server = Server()

# client = mqtt.Client()
# client.connect("10.42.0.43", 1883, 60)


def on_message(client, userdata, message):
    carga = unpack("iid", message.payload)
    robot.infrared_sensors = carga[:2]
    # print("Received message:", carga[:2], time.time() - float(carga[2]))


def on_connect(client, userdata, flags, rc):
    #print("The robots are connected with result code", str(rc))
    client.subscribe("topic/sensors")


server.client.on_connect = on_connect
server.client.on_message = on_message
server.client.loop_start()


def main():
    try:
        while True:
            pass
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
