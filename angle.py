import cv2
import math

# ArUco setup
aruco_dict = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(
    aruco_dict,
    parameters
)

# Camera
cap = cv2.VideoCapture(1)


def get_marker_angle(marker_corners):
    # Get the 4 corners
    c = marker_corners[0]

    # Top-left and top-right corners
    p1 = c[0]
    p2 = c[1]

    # Direction vector
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    # Calculate angle
    angle = math.degrees(math.atan2(dy, dx))

    return angle


print("Press Q to quit")

while True:

    ret, frame = cap.read()

    if not ret:
        break

    corners, ids, rejected = detector.detectMarkers(frame)

    if ids is not None:

        cv2.aruco.drawDetectedMarkers(
            frame, corners, ids
        )

        for i in range(len(ids)):

            marker_id = int(ids[i].flatten()[0])

            # Marker center
            c = corners[i][0]

            center_x = int(c[:, 0].mean())
            center_y = int(c[:, 1].mean())

            # Marker angle
            angle = get_marker_angle(corners[i])

            # Print to terminal
            print(
                f"ID: {marker_id} | "
                f"Position: ({center_x}, {center_y}) | "
                f"Angle: {angle:.1f}°"
            )

            # Show angle on screen
            cv2.putText(
                frame,
                f"Angle: {angle:.1f} deg",
                (center_x + 10, center_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            # Show marker center
            cv2.circle(
                frame,
                (center_x, center_y),
                5,
                (0, 0, 255),
                -1
            )

    cv2.imshow("Marker Angle Test", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()