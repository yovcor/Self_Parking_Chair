import cv2

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

marker_size_px = 400  # larger pixel size for cleaner printing
border_px = 50         # white border around the marker

for marker_id in range(5):  # generates markers with IDs 0-4
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)

    # Add a white border (quiet zone) around the marker
    bordered = cv2.copyMakeBorder(
        marker_img,
        border_px, border_px, border_px, border_px,
        cv2.BORDER_CONSTANT,
        value=255
    )

    cv2.imwrite(f"marker_{marker_id}.png", bordered)

print("Markers generated with white border. Print each at around 6-8cm wide for reliable detection.")