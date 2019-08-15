#!/usr/bin/env python3
import paho.mqtt.client as mqtt
class Server:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.connect("10.42.0.43", 1883, 60)

    @property
    def get_client(self):
        return self.client

