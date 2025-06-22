import cv2
import numpy as np

# Load AprilTag dictionary
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
detector = cv2.aruco.ArucoDetector(dictionary)

# Define real-world tag size (meters)
KNOWN_WIDTH = 0.275  # Adjust based on your tag size

# Camera focal length (calibrate using Step 1)
FOCAL_LENGTH = 560  # Replace with your calculated F value

# Open camera
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while cap.isOpened():
    ret, frame = cap.read()
    
    if not ret:
        break

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        for i, corner in enumerate(corners):
            # Get bounding box width in pixels
            tag_width_pixels = np.linalg.norm(corner[0][0] - corner[0][1])

            # Estimate distance
            distance = (KNOWN_WIDTH * FOCAL_LENGTH) / tag_width_pixels

            # Display ID and distance
            center = tuple(np.mean(corner[0], axis=0).astype(int))
            cv2.putText(frame, f"ID: {ids[i][0]}", (center[0] - 20, center[1] - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            cv2.putText(frame, f"Distance: {distance:.2f}m", (center[0] - 20, center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Show frame
    cv2.imshow("AprilTag Distance Estimation", frame)
    if cv2.waitKey(1) & 0xFF == 27:  # Press ESC to exit
        break

cap.release()
cv2.destroyAllWindows()
