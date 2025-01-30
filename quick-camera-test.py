import cv2

# Open the camera device
cap = cv2.VideoCapture('/dev/grey', cv2.CAP_V4L2)

if not cap.isOpened():
    print("Error: Could not open camera")
else:
    ret, frame = cap.read()  # Capture a single frame
    if ret:
        filename = "captured_image.jpg"
        cv2.imwrite(filename, frame)  # Save the image
        print(f"Image saved as {filename}")
    else:
        print("Error: Could not read frame")

    cap.release()  # Release the camera
