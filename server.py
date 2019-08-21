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


def on_publish(client,userdata,result):             #create function for callback
    print("data published \n")
    pass

client = mqtt.Client()
client.on_publish = on_publish

client.connect("localhost", 1883, 60)

ultrasonic = ev3.UltrasonicSensor("in3")
color_sensors = Duo(ev3.ColorSensor("in1"), ev3.ColorSensor("in2"))

client.loop_start()


try:
    while True:
        message = pack("iiid", ultrasonic.value(), color_sensors.left.color, color_sensors.right.color, time.time())
        client.publish("topic/sensors", message, qos=0)
        print(unpack("iiid", message))
        time.sleep(0.05)

except KeyboardInterrupt:
    pass


client.loop_end()
client.disconnect()
