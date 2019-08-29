import paho.mqtt.client as mqtt
from struct import *

def on_message(client, userdata, message):
    print("mensagem recebida")
    payload = unpack("iiid", message.payload)
    #robot.color_sensors = payload[1:3]
    #robot.ultrasonic_sensor = payload[0]
    print(payload)


def on_connect(client, userdata, flags, rc):
    print("The robots are connected with result code", str(rc))
    client.subscribe("topic/sensors")


def on_disconnect(client, userdata, rc):
    print("The robots are FAILEEDD with result code", str(rc))


client = mqtt.Client()
client.connect("169.254.35.189", 1883, 60)
client.loop_start()
client.on_connect = on_connect
client.on_message = on_message

client.loop_start()


while True: pass
