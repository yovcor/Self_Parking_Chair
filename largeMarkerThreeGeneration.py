import cv2
import cv2.aruco as aruco

# ============================================================
# ARUCO CONFIGURATION (DICT_4X4_50 & Marker ID 3)
# ============================================================
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
MARKER_ID = 3

# 300 DPI Resolution Calculation for 15cm x 15cm
# 15 cm = 5.9 inches -> 5.9 * 300 DPI ≈ 1770 Pixels
TOTAL_SIZE_PX = 1770  
BORDER_PX = 200       # ~1.7cm White Border (Quiet Zone)
MARKER_SIZE_PX = TOTAL_SIZE_PX - (2 * BORDER_PX)

# 1. Generate ArUco Marker Image
marker_img = aruco.generateImageMarker(ARUCO_DICT, MARKER_ID, MARKER_SIZE_PX)

# 2. Add White Border around Marker (Essential for camera detection)
final_marker = cv2.copyMakeBorder(
    marker_img,
    BORDER_PX, BORDER_PX, BORDER_PX, BORDER_PX,
    cv2.BORDER_CONSTANT,
    value=255
)

# 3. Save as PNG
output_filename = f"aruco_marker_id{MARKER_ID}_15x15cm.png"
cv2.imwrite(output_filename, final_marker)

print(f"[SUCCESS]: Created '{output_filename}' successfully!")