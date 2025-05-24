import cv2

# Initialize the video capture object
cap = cv2.VideoCapture(0)

# Initialize the QR code detector
detector = cv2.QRCodeDetector()

while True:
    # Read a frame from the video stream
    ret, frame = cap.read()

    if not ret:
        break

# Detect QR codes in the frame
    data, bbox, _ = detector.detectAndDecode(frame)

# If a QR code is detected, draw a bounding box around it
    if data:
        if bbox is not None:
            for i in range(len(bbox)):
            # Convert bounding box points to integers
                pt1 = tuple(map(int, bbox[i][0]))
                pt2 = tuple(map(int, bbox[(i + 1) % len(bbox)][0]))
                cv2.line(frame, pt1, pt2, color=(255, 0, 0), thickness=2)
        print("QR Code detected:", data)
    else:
        print("QR code not detected")

# Display the resulting frame
    cv2.imshow("QR Code Detection", frame)

# Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close all windows

cv2.destroyAllWindows()