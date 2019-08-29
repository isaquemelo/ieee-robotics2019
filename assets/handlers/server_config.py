#!/usr/bin/env python3
import paho.mqtt.client as mqtt
from struct import *

class Server:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.connect("169.254.96.252", 1883, 60)

    @property
    def get_client(self):
        return self.client

