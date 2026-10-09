import paho.mqtt.client as mqtt
import time

BROKER = "ip"
PORT = 1883
TOPIC = "chair/commands"

client = mqtt.Client()

client.connect(BROKER, PORT, 60)

# Start MQTT communication
client.loop_start()

print("Waiting for connection...")
time.sleep(2)

print("Sending ROTATE_CW:150")

result = client.publish(TOPIC, "ROTATE_CW:150")
result.wait_for_publish()

print("Command sent!")

# Keep chair rotating for 5 seconds
time.sleep(5)

print("Sending STOP")

result = client.publish(TOPIC, "STOP")
result.wait_for_publish()

time.sleep(1)

client.loop_stop()
client.disconnect()