# Self-Parking Chair

An IoT-based intelligent self-parking chair (CO3302 hardware project). On a hand clap, the chair finds its own way back to a fixed "home" spot: an overhead camera tracks the chair using an ArUco marker, an A* planner routes around obstacles, and an ESP32 drives the mecanum-wheel base over MQTT. If the chair's ultrasonic sensors detect something the camera missed, it performs a wall-following detour and then re-plans.

## How it works

1. **Wait for clap.** The ESP32 listens for a clap and publishes `CLAP_DETECTED`.
2. **Locate the chair.** An overhead camera detects the ArUco marker (DICT_4X4_50, ID 3) on the chair to get its position and heading.
3. **Plan a path.** The frame is thresholded to find obstacles, which are inflated by the chair radius plus a safety buffer (20 cm total). A* runs on a downsampled 64x48 grid, and the result is simplified into waypoints.
4. **Drive.** For each waypoint the chair first rotates to face it (if the heading error is over 25°), then drives forward with speed scaled by distance.
5. **Handle surprises.** If the ESP32 reports `OBSTACLE`, the chair stops, reads its side sensors, turns toward the more open side, follows the obstacle wall until the edge is passed, then re-plans from its new position.
6. **Park.** At home, the chair aligns to the target heading and sends `RESET_CLAP`.

```
 Overhead camera ──> navigate.py (OpenCV + A*) ──MQTT──> ESP32 ──> L298N drivers ──> mecanum wheels
                              ^                             |
                              └────── status / sensors ─────┘
```

## Hardware

- ESP32 microcontroller
- Mecanum wheel base with L298N motor drivers
- Ultrasonic sensors (front, left, right)
- Clap / sound sensor
- LiPo battery with LM2596 buck converter
- Overhead USB camera (640x480) and a printed ArUco marker (ID 3) on top of the chair
- A WiFi network and an MQTT broker (e.g. Mosquitto) reachable by both the PC and the ESP32

## Repository structure

```
SelfParkingChair/
├── navigate.py          # Main homing controller (camera + A* + MQTT)
├── <test_*.py>          # Early test scripts used to verify the robot base
├── firmware/            # ESP32 Arduino sketch(es)
├── requirements.txt
└── README.md
```

> Update the file names above to match your actual test scripts and firmware folder.

## Getting started

### Prerequisites

- Python 3.9+
- An MQTT broker running on your network
- ESP32 flashed with the chair firmware (see `firmware/`)

### Install

```bash
git clone https://github.com/<your-username>/SelfParkingChair.git
cd SelfParkingChair
python -m venv venv
venv\Scripts\activate        # Windows
# source venv/bin/activate   # Linux / macOS
pip install -r requirements.txt
```

### Configure

Edit the constants at the top of `navigate.py`:

| Setting | Default | Meaning |
|---|---|---|
| `MQTT_BROKER` / `MQTT_PORT` | `192.168.1.101` / `1883` | Address of your MQTT broker |
| `HOME_X`, `HOME_Y` | `320`, `420` | Home position in camera pixels |
| `HOME_THETA` | `0.0` | Target parking heading in degrees |
| `HOME_TOLERANCE_PX` | `45` | Radius of the home circle |
| `WAYPOINT_TOLERANCE` | `35` | Distance at which a waypoint counts as reached |
| `CHAIR_RADIUS_CM`, `EXTRA_SAFETY_CM` | `15`, `5` | Obstacle inflation buffer |
| `CHAIR_MARKER_ID` | `3` | ArUco ID attached to the chair |

The camera index is auto-detected (tries 1, 0, then 2).

### Run

1. Power on the chair and confirm the ESP32 is connected to WiFi and the broker.
2. Place the chair with its ArUco marker visible to the camera.
3. Start the controller:
   ```bash
   python navigate.py
   ```
4. Clap once. The chair plans a path and drives home.

Press `Ctrl+C` in the terminal to stop the script. The ESP32 should also stop on its own if commands cease.

## MQTT protocol

The PC publishes commands to `chair/commands` and listens on `chair/status`.

| Direction | Message | Meaning |
|---|---|---|
| PC → chair | `FORWARD:<pwm>` | Drive forward at the given PWM |
| PC → chair | `ROTATE_CW:<deg>:<pwm>` / `ROTATE_ACW:<deg>:<pwm>` | Rotate by an angle |
| PC → chair | `STOP` | Stop all motors |
| PC → chair | `GET_SENSORS` | Request ultrasonic readings |
| PC → chair | `RESET_CLAP` | Re-arm the clap trigger |
| Chair → PC | `CLAP_DETECTED` | Start signal |
| Chair → PC | `FRONT:<cm>,LEFT:<cm>,RIGHT:<cm>` | Sensor readings |
| Chair → PC | `OBSTACLE` | Something is blocking the path |
| Chair → PC | `DONE` | Rotation finished |

## Troubleshooting

- **"Chair ArUco Marker ID 3 not detected":** check lighting, marker size and that nothing is covering it.
- **Wrong camera opens:** change the index order in `get_working_camera()`.
- **No MQTT connection:** confirm the broker IP, that the PC and ESP32 are on the same network, and that port 1883 is not blocked by a firewall.
- **Chair drives to the wrong spot:** recalibrate `HOME_X` / `HOME_Y` for your camera mounting position.

## Known limitations

- Obstacle detection from the camera uses a simple brightness threshold, so it depends on lighting and floor colour.
- The chair is tracked in pixel space, so changing the camera height or angle needs recalibration.
- Home position and tolerances are hard-coded.

## Team

Built for CO3302, Department of Computer Engineering, University of Sri Jayewardenepura.

- `<Your name>`
- `<Teammate name>`
- `<Teammate name>`

## License

Add a license of your choice (MIT is a common default for coursework projects) or remove this section.
