import paho.mqtt.client as mqtt
from struct import *
from ev3dev import ev3



while True:
    print(ev3.InfraredSensor('in1').value())
