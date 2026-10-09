import paho.mqtt.client as mqtt
import time

# ============================================================
# MQTT SETTINGS
# ============================================================

broker = "ip"
port = 1883
topic = "chair/commands"


# ============================================================
# MQTT CLIENT
# ============================================================

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)


print("Connecting to MQTT broker...")

client.connect(broker, port, 60)

# Keep MQTT network communication running
client.loop_start()

print("Connected to MQTT broker.")
print("")


# ============================================================
# SEND COMMAND
# ============================================================

def send(command, speed=None):
    """
    Send a command to the ESP32.

    Format:
        FORWARD:50
        FORWARD:100
        BACKWARD:100
        LEFT:100
        RIGHT:100
        ROTATE_CW:100
        ROTATE_ACW:100
        STOP
    """

    if speed is not None:
        msg = f"{command}:{speed}"
    else:
        msg = command

    result = client.publish(topic, msg, qos=1)

    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        print(f"Sent: {msg}")
    else:
        print(f"FAILED to send: {msg}")

    # Small gap between MQTT messages
    time.sleep(0.2)


# ============================================================
# START TEST
# ============================================================

print("======================================")
print(" ESP32-WROOM-32D MOTOR TEST")
print("======================================")

print("Make sure:")
print("1. ESP32 is powered")
print("2. ESP32 is connected to WiFi")
print("3. ESP32 is connected to MQTT")
print("4. Motor battery is connected")
print("5. Chair is in a safe position")
print("")

input("Press Enter to begin...")


# ============================================================
# TEST 1 — FORWARD SLOW
# ============================================================

print("\nTEST 1: FORWARD - Speed 50")

send("FORWARD", 50)

time.sleep(3)

send("STOP")

time.sleep(2)


# ============================================================
# TEST 2 — FORWARD MEDIUM
# ============================================================

print("\nTEST 2: FORWARD - Speed 100")

send("FORWARD", 100)

time.sleep(3)

send("STOP")

time.sleep(2)


# ============================================================
# TEST 3 — FORWARD FAST
# ============================================================

print("\nTEST 3: FORWARD - Speed 150")

send("FORWARD", 150)

time.sleep(3)

send("STOP")

time.sleep(2)


# ============================================================
# TEST 4 — ROTATE CLOCKWISE
# ============================================================

print("\nTEST 4: ROTATE CLOCKWISE - Speed 100")

send("ROTATE_CW", 100)

time.sleep(3)

send("STOP")

time.sleep(2)


# ============================================================
# TEST 5 — ROTATE ANTICLOCKWISE
# ============================================================

print("\nTEST 5: ROTATE ANTICLOCKWISE - Speed 100")

send("ROTATE_ACW", 150)

time.sleep(3)

send("STOP")

time.sleep(2)


# ============================================================
# TEST 6 — BACKWARD
# ============================================================

print("\nTEST 6: BACKWARD - Speed 150")

send("BACKWARD", 100)

time.sleep(3)

send("STOP")

time.sleep(2)


# ============================================================
# TEST COMPLETE
# ============================================================

print("\n======================================")
print(" ALL TESTS COMPLETE")
print("======================================")


# Stop MQTT background thread
client.loop_stop()

# Disconnect
client.disconnect()

print("MQTT disconnected.")