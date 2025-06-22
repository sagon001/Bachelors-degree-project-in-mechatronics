import cv2
import cv2.aruco as aruco

#Define the AprilTag dictionary (AprilTag 36h11 is commonly used)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
aruco_params = aruco.DetectorParameters()

# Start video capture
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Get frame dimensions
    height, width, _ = frame.shape
    center_x, center_y = width // 2, height // 2

    # Convert frame to grayscale for better processing
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is not None:
        for i in range(len(ids)):
            # Get the corner points of the detected tag
            corner_points = corners[i][0]  # Extract first marker points

            # Calculate the center of the detected AprilTag
            tag_center_x = int((corner_points[0][0] + corner_points[2][0]) / 2)
            tag_center_y = int((corner_points[0][1] + corner_points[2][1]) / 2)

            # Calculate the pixel distance from the image center
            dx = tag_center_x - center_x
            dy = tag_center_y - center_y

            # Draw the marker outline and its center
            aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.circle(frame, (tag_center_x, tag_center_y), 5, (0, 0, 255), -1)

            # Display the distance
            text = f"dx: {dx}px, dy: {dy}px"
            cv2.putText(frame, text, (tag_center_x + 10, tag_center_y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Draw a cross at the image center
    cv2.drawMarker(frame, (center_x, center_y), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)

    # Show the output
    cv2.imshow('AprilTag Tracking', frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
