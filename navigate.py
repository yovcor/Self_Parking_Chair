import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time
import threading
import heapq
import paho.mqtt.client as mqtt

# ============================================================
# MQTT CONFIGURATION
# ============================================================
MQTT_BROKER = "ip"
MQTT_PORT = 1883
TOPIC_COMMANDS = "chair/commands"
TOPIC_STATUS = "chair/status"

status_event = threading.Event()
last_status = ""

# ============================================================
# FIXED HOME POSITION & TARGET TOLERANCES
# ============================================================
HOME_X = 320        # Target Home X position (Pixels)
HOME_Y = 420        # Target Home Y position (Pixels)
HOME_THETA = 0.0    # Target Home Angle (Degrees)

HOME_TOLERANCE_PX = 45  # Target Circle Radius (45px)
WAYPOINT_TOLERANCE = 35 # Waypoint acceptance radius
MAX_LOST_FRAMES = 5     # Position memory frames

CHAIR_RADIUS_CM = 15.0   # Chair physical radius
EXTRA_SAFETY_CM = 5.0     # Extra safety buffer
TOTAL_BUFFER_CM = CHAIR_RADIUS_CM + EXTRA_SAFETY_CM # Total 20 cm clearance

current_visual_path = []
current_obstacle_mask = None

# ============================================================
# ARUCO DETECTOR SETUP (DICT_4X4_50, ID 3)
# ============================================================
CHAIR_MARKER_ID = 3
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
ARUCO_PARAMS = aruco.DetectorParameters()
detector = aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

def get_working_camera():
    print("[INFO] Testing available camera indexes...")
    for idx in [1, 0, 2]:
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                print(f"[SUCCESS] Active Camera found on Index {idx}")
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                return cap
            cap.release()
    return None

def get_chair_pose(corners, ids):
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == CHAIR_MARKER_ID:
                pts = corners[i][0]
                cx = int((pts[0][0] + pts[2][0]) / 2)
                cy = int((pts[0][1] + pts[2][1]) / 2)
                dx = pts[1][0] - pts[0][0]
                dy = pts[1][1] - pts[0][1]
                angle = math.degrees(math.atan2(dy, dx))
                return cx, cy, angle
    return None, None, None

# ============================================================
# A* PATHFINDING ENGINE
# ============================================================
def astar(grid_matrix, start, goal):
    rows, cols = grid_matrix.shape
    start_node = (start[1], start[0])
    goal_node = (goal[1], goal[0])
    
    if grid_matrix[start_node] == 1 or grid_matrix[goal_node] == 1:
        return [start, goal]

    open_set = []
    heapq.heappush(open_set, (0, start_node))
    came_from = {}
    g_score = {start_node: 0}
    f_score = {start_node: math.hypot(start_node[0]-goal_node[0], start_node[1]-goal_node[1])}
    
    directions = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
    
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal_node:
            path = []
            while current in came_from:
                path.append((current[1], current[0]))
                current = came_from[current]
            path.append((start_node[1], start_node[0]))
            path.reverse()
            return path
            
        for dr, dc in directions:
            neighbor = (current[0] + dr, current[1] + dc)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if grid_matrix[neighbor] == 1:
                    continue
                cost = 1.414 if (dr != 0 and dc != 0) else 1.0
                tentative_g = g_score[current] + cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = math.hypot(neighbor[0]-goal_node[0], neighbor[1]-goal_node[1])
                    f_score[neighbor] = tentative_g + h
                    heapq.heappush(open_set, (tentative_g + h, neighbor))
                    
    return [start, goal]

def simplify_path(path, step=10):
    if len(path) <= 2:
        return path
    simplified = [path[0]]
    for i in range(step, len(path) - 1, step):
        simplified.append(path[i])
    simplified.append(path[-1])
    return simplified

def compute_global_astar_path(frame, chair_x, chair_y, aruco_pixel_width=45):
    global current_obstacle_mask
    
    cm_per_pixel = 15.0 / max(10, aruco_pixel_width)
    buffer_pixels = int(TOTAL_BUFFER_CM / cm_per_pixel)
    buffer_pixels = max(20, min(80, buffer_pixels))
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY_INV)
    
    cv2.circle(thresh, (chair_x, chair_y), buffer_pixels + 10, 0, -1)
    cv2.circle(thresh, (HOME_X, HOME_Y), buffer_pixels + 10, 0, -1)
    
    kernel = np.ones((buffer_pixels, buffer_pixels), np.uint8)
    dilated_mask = cv2.dilate(thresh, kernel, iterations=1)
    current_obstacle_mask = dilated_mask.copy()
    
    grid_small = cv2.resize(dilated_mask, (64, 48), interpolation=cv2.INTER_NEAREST)
    grid_matrix = np.where(grid_small > 0, 1, 0)
    
    start_grid = (int(chair_x / 10), int(chair_y / 10))
    goal_grid = (int(HOME_X / 10), int(HOME_Y / 10))
    
    grid_path = astar(grid_matrix, start_grid, goal_grid)
    full_path = [(x * 10 + 5, y * 10 + 5) for (x, y) in grid_path]
    full_path[-1] = (HOME_X, HOME_Y)
    
    smoothed_path = simplify_path(full_path, step=10)
    print(f"[A* CLEARANCE]: Inflated obstacles by {TOTAL_BUFFER_CM}cm ({buffer_pixels}px) for chair radius.")
    return smoothed_path

# ============================================================
# UNIFIED FRAME PROCESSOR
# ============================================================
def process_and_draw_frame(cap, memory_data):
    ret, frame = cap.read()
    if not ret or frame is None:
        return None, None, None, None

    last_cx, last_cy, last_theta, lost_count = memory_data

    corners, ids, _ = detector.detectMarkers(frame)
    raw_cx, raw_cy, raw_theta = get_chair_pose(corners, ids)

    if raw_cx is not None:
        cx, cy, theta_c = raw_cx, raw_cy, raw_theta
        last_cx, last_cy, last_theta = cx, cy, theta_c
        lost_count = 0
        aruco.drawDetectedMarkers(frame, corners, ids)
    elif last_cx is not None and lost_count < MAX_LOST_FRAMES:
        cx, cy, theta_c = last_cx, last_cy, last_theta
        lost_count += 1
        cv2.putText(frame, f"MEMORY MODE ({lost_count}/{MAX_LOST_FRAMES})",
                    (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    else:
        cx, cy, theta_c = None, None, None
        last_cx, last_cy, last_theta = None, None, None

    if current_obstacle_mask is not None:
        red_overlay = np.zeros_like(frame)
        red_overlay[current_obstacle_mask > 0] = (0, 0, 255)
        frame = cv2.addWeighted(frame, 0.85, red_overlay, 0.15, 0)

    if len(current_visual_path) > 1:
        for i in range(len(current_visual_path) - 1):
            pt1 = current_visual_path[i]
            pt2 = current_visual_path[i+1]
            cv2.line(frame, pt1, pt2, (0, 255, 0), 3)
            cv2.circle(frame, pt1, 5, (0, 255, 255), -1)

    cv2.circle(frame, (HOME_X, HOME_Y), HOME_TOLERANCE_PX, (0, 255, 0), 2)
    cv2.circle(frame, (HOME_X, HOME_Y), 3, (0, 255, 0), -1)
    cv2.putText(frame, "HOME", (HOME_X - 20, HOME_Y - 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    if cx is not None:
        cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)

    cv2.imshow("Homing Camera View", frame)
    cv2.waitKey(1)

    updated_memory = (last_cx, last_cy, last_theta, lost_count)
    return frame, cx, cy, theta_c, updated_memory

def sleep_with_camera(cap, duration, memory_data):
    end_time = time.time() + duration
    while time.time() < end_time:
        _, _, _, _, memory_data = process_and_draw_frame(cap, memory_data)
    return memory_data

# ============================================================
# MQTT CALLBACKS & HELPERS
# ============================================================
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[MQTT] Connected to Broker successfully!")
        client.subscribe(TOPIC_STATUS)
    else:
        print(f"[MQTT] Connection failed with code {rc}")

def on_message(client, userdata, msg):
    global last_status
    last_status = msg.payload.decode("utf-8").strip()
    status_event.set()

def get_sensor_readings(client, cap, memory_data):
    global last_status
    status_event.clear()
    last_status = ""
    client.publish(TOPIC_COMMANDS, "GET_SENSORS")
    
    start_t = time.time()
    while time.time() - start_t < 1.0:
        if last_status.startswith("FRONT:"):
            try:
                parts = last_status.split(",")
                f = float(parts[0].split(":")[1])
                l = float(parts[1].split(":")[1])
                r = float(parts[2].split(":")[1])
                return f, l, r
            except Exception:
                break
        memory_data = sleep_with_camera(cap, 0.05, memory_data)
        
    return 999.0, 999.0, 999.0

def rotate_and_wait(client, cap, direction, angle, memory_data, speed=115):
    global last_status
    status_event.clear()
    last_status = ""

    angle_val = float(angle)
    angle_val = min(180.0, max(5.0, angle_val))
    calculated_timeout = max(10, int(angle_val / 8.0) + 6)

    cmd = f"ROTATE_{direction}:{angle_val:.1f}:{speed}"
    print(f"\n[ACTION]: Rotating {direction} by {angle_val:.1f}°")
    client.publish(TOPIC_COMMANDS, cmd)

    start_time = time.time()
    while time.time() - start_time < calculated_timeout:
        if status_event.is_set():
            break
        _, _, _, _, memory_data = process_and_draw_frame(cap, memory_data)

    memory_data = sleep_with_camera(cap, 0.2, memory_data)
    return last_status == "DONE", memory_data

def bypass_obstacle(client, cap, memory_data):
    global last_status
    print("\n==================================================")
    print(" [OBSTACLE DETECTED]: INITIATING 30° SENSOR DETOUR ")
    print("==================================================")

    f, l, r = get_sensor_readings(client, cap, memory_data)
    print(f"[SENSORS]: Front={f:.1f}cm, Left={l:.1f}cm, Right={r:.1f}cm")

    if r >= l:
        turn_dir = "CW"
        track_side = "LEFT"
    else:
        turn_dir = "ACW"
        track_side = "RIGHT"

    print(f"[ACTION]: Turning 90° {turn_dir} (Tracking {track_side} wall)...")
    _, memory_data = rotate_and_wait(client, cap, turn_dir, 90.0, memory_data, speed=115)

    sf, sl, sr = get_sensor_readings(client, cap, memory_data)
    baseline_dist = sl if track_side == "LEFT" else sr
    if baseline_dist > 900: baseline_dist = 22.0

    print(f"[30° SENSOR BASELINE]: Initial Wall Distance = {baseline_dist:.1f}cm")

    wall_cleared = False
    step_count = 0
    max_steps = 8

    while not wall_cleared and step_count < max_steps:
        step_count += 1
        print(f"[WALL STEP {step_count}]: Advancing along obstacle side wall...")

        client.publish(TOPIC_COMMANDS, "FORWARD:120")
        memory_data = sleep_with_camera(cap, 0.25, memory_data)
        client.publish(TOPIC_COMMANDS, "STOP")
        memory_data = sleep_with_camera(cap, 0.20, memory_data)

        sf, sl, sr = get_sensor_readings(client, cap, memory_data)
        current_side_dist = sl if track_side == "LEFT" else sr
        print(f"[30° CHECK]: Side Distance = {current_side_dist:.1f}cm")

        if current_side_dist > 40.0 or (current_side_dist - baseline_dist) > 12.0:
            wall_cleared = True
            print("[WALL END DETECTED]: Side sensor has passed obstacle edge!")

    print("[CHAIR CLEARANCE]: Pushing 0.50s extra forward to clear chair rear radius...")
    client.publish(TOPIC_COMMANDS, "FORWARD:125")
    memory_data = sleep_with_camera(cap, 0.50, memory_data)
    client.publish(TOPIC_COMMANDS, "STOP")
    memory_data = sleep_with_camera(cap, 0.30, memory_data)

    last_status = ""
    print("[DETOUR SUCCESS]: Obstacle wall and chair diameter cleared safely!\n")
    return memory_data

# ============================================================
# HOMING CONTROL LOOP WITH CORRECT INITIAL ROTATION
# ============================================================
def run_auto_homing(client, cap):
    global last_status, current_visual_path
    print("\n==========================================")
    print("    STARTING A* SMART CHAIR HOMING LOOP    ")
    print("==========================================")

    memory_data = (None, None, None, 0)
    
    frame, cx, cy, theta_c, memory_data = process_and_draw_frame(cap, memory_data)
    if cx is None:
        print("[ERROR] Cannot start homing: Chair ArUco Marker ID 3 not detected!")
        return

    # GENERATE STATIC A* PATH BEFORE MOVEMENT
    waypoints = compute_global_astar_path(frame, cx, cy)
    current_visual_path = waypoints
    waypoint_index = 0

    last_sent_speed = 0
    is_moving = False

    while waypoint_index < len(waypoints):
        target_wp = waypoints[waypoint_index]

        frame, cx, cy, theta_c, memory_data = process_and_draw_frame(cap, memory_data)

        if frame is None:
            print("[ERROR] Camera frame capture failed!")
            break

        if cx is not None:
            distance = math.sqrt((target_wp[0] - cx)**2 + (target_wp[1] - cy)**2)
            target_heading = math.degrees(math.atan2(target_wp[1] - cy, target_wp[0] - cx))
            angle_error = target_heading - theta_c

            while angle_error > 180: angle_error -= 360
            while angle_error < -180: angle_error += 360

            # ----------------------------------------------------
            # STEP 1: COURSE CORRECTION (ROTATES FIRST BEFORE MOVING FORWARD)
            # ----------------------------------------------------
            if abs(angle_error) > 25: # Fixed: Always rotate first if angle error > 25 deg
                if is_moving:
                    client.publish(TOPIC_COMMANDS, "STOP")
                    is_moving = False
                    time.sleep(0.1)

                direction = "CW" if angle_error > 0 else "ACW"
                safe_angle = min(180.0, abs(angle_error))
                _, memory_data = rotate_and_wait(client, cap, direction, safe_angle, memory_data, speed=115)
                continue

            # ----------------------------------------------------
            # STEP 2: DRIVE FORWARD TO CURRENT WAYPOINT
            # ----------------------------------------------------
            elif distance > WAYPOINT_TOLERANCE:
                target_speed = int(min(145, max(110, distance * 0.5)))

                if abs(target_speed - last_sent_speed) >= 5 or not is_moving:
                    client.publish(TOPIC_COMMANDS, f"FORWARD:{target_speed}")
                    last_sent_speed = target_speed
                    is_moving = True
                    print(f"[A* DRIVE]: Waypoint {waypoint_index+1}/{len(waypoints)} | Dist={distance:.1f}px -> Speed={target_speed} PWM")

                if last_status == "OBSTACLE":
                    client.publish(TOPIC_COMMANDS, "STOP")
                    is_moving = False
                    memory_data = bypass_obstacle(client, cap, memory_data)
                    
                    frame, cx, cy, theta_c, memory_data = process_and_draw_frame(cap, memory_data)
                    if cx is not None:
                        waypoints = compute_global_astar_path(frame, cx, cy)
                        current_visual_path = waypoints
                        waypoint_index = 0
                    continue

                cv2.waitKey(1)
                continue

            # ----------------------------------------------------
            # STEP 3: WAYPOINT REACHED -> ADVANCE TO NEXT
            # ----------------------------------------------------
            else:
                print(f"[WAYPOINT {waypoint_index+1} REACHED!]: Progressing to next waypoint...")
                waypoint_index += 1
                if is_moving:
                    client.publish(TOPIC_COMMANDS, "STOP")
                    is_moving = False

    # ----------------------------------------------------
    # STEP 4: FINAL PARKING AT HOME
    # ----------------------------------------------------
    client.publish(TOPIC_COMMANDS, "STOP")
    is_moving = False
    print("\n[HOME AREA REACHED!]: Aligning final orientation...")

    if cx is not None:
        final_angle_err = HOME_THETA - theta_c
        while final_angle_err > 180: final_angle_err -= 360
        while final_angle_err < -180: final_angle_err += 360

        if abs(final_angle_err) > 15:
            direction = "CW" if final_angle_err > 0 else "ACW"
            safe_angle = min(180.0, abs(final_angle_err))
            _, memory_data = rotate_and_wait(client, cap, direction, safe_angle, memory_data, speed=100)

    print("\n==========================================")
    print(" SUCCESS: CHAIR PARKED AT HOME POSITION! ")
    print("==========================================")
    client.publish(TOPIC_COMMANDS, "STOP")
    client.publish(TOPIC_COMMANDS, "RESET_CLAP")

# ============================================================
# MAIN FUNCTION
# ============================================================
def main():
    global last_status
    client = mqtt.Client(client_id="Python_Homing_Controller")
    client.on_connect = on_connect
    client.on_message = on_message

    print("Connecting to MQTT Broker...")
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
    except Exception as e:
        print(f"[ERROR] MQTT Connection Failed: {e}")
        return

    time.sleep(1)

    cap = get_working_camera()
    if cap is None:
        print("[FATAL] No functional camera detected on indexes 1, 0, or 2!")
        return

    print("\n==================================================")
    print(" [WAITING FOR CLAP]: Clap your hands once to start!")
    print("==================================================")

    memory_data = (None, None, None, 0)
    while True:
        if "CLAP_DETECTED" in last_status:
            print("\n[CLAP RECEIVED!]: Starting A* Path Planning & Chair Homing...")
            last_status = ""
            break

        frame, _, _, _, memory_data = process_and_draw_frame(cap, memory_data)
        if frame is not None:
            cv2.putText(frame, "WAITING FOR CLAP TO START...", (30, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("Homing Camera View", frame)
            cv2.waitKey(1)
        time.sleep(0.02)

    run_auto_homing(client, cap)

    cap.release()
    cv2.destroyAllWindows()
    client.loop_stop()
    client.disconnect()

if __name__ == "__main__":
    main()