import cv2

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

cap = cv2.VideoCapture(1)  # change index if needed

cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # disable autofocus
# cap.set(cv2.CAP_PROP_FOCUS, 0)      # set to infinity focus (best for overhead/distant view)

actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"Camera resolution: {int(actual_w)} x {int(actual_h)}")

# Confirm what resolution was actually set
actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"Camera resolution set to: {int(actual_w)} x {int(actual_h)}")
print("Move the marker to the HOME position and note the coordinates shown on screen.")
print("Press Q to quit once you have noted the home coordinates.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Cannot read from camera.")
        break

    corners, ids, rejected = detector.detectMarkers(frame)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for i, marker_id in enumerate(ids):
            c = corners[i][0]
            center_x = int(c[:, 0].mean())
            center_y = int(c[:, 1].mean())

            # Draw center point
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

            # Show coordinates on screen
            cv2.putText(frame, f"ID:{marker_id} X:{center_x} Y:{center_y}",
                       (center_x - 50, center_y - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            print(f"Marker {marker_id}: X={center_x}, Y={center_y}")

    # Draw crosshair at center of frame for reference
    h, w = frame.shape[:2]
    cv2.line(frame, (w//2 - 20, h//2), (w//2 + 20, h//2), (255, 0, 0), 1)
    cv2.line(frame, (w//2, h//2 - 20), (w//2, h//2 + 20), (255, 0, 0), 1)
    cv2.putText(frame, f"Frame center: ({w//2}, {h//2})",
               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    cv2.putText(frame, f"Resolution: {int(actual_w)}x{int(actual_h)}",
               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    cv2.imshow("Home Position Finder", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()