#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import ev3dev.ev3 as ev3
from struct import *
from datetime import datetime, timedelta
import time


class Duo:
    def __init__(self, sensor_left, sensor_right, sensor_back=None):
        self.left = sensor_left
        self.right = sensor_right
        self.values = (self.left, self.right)


client = mqtt.Client()


client.connect("localhost", 1883, 60)

infrared_sensor = Duo(ev3.InfraredSensor('in1'), ev3.InfraredSensor('in2'))
#infrared_sensor.left.

client.loop_start()

try:
    while True:
        message = pack("iid", infrared_sensor.left.value(), infrared_sensor.right.value(), time.time())
        client.publish("topic/sensors", message, qos=0)
        print(unpack("iid", message))
        time.sleep(0.1)

except KeyboardInterrupt:
    pass


client.loop_end()
client.disconnect()
