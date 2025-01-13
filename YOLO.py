from ultralytics import YOLO
import cv2

# Load a YOLO model (pre-trained YOLOv8 nano model)
model = YOLO('yolov8n.pt')  # Replace with your model if needed

# Load an image
image_path = "image.png"  # Replace with your image path
image = cv2.imread(image_path)

# Run object detection
results = model(image)

# Extract the first result (since we're working with one image)
result = results[0]

# Iterate through the bounding boxes
for box in result.boxes:
    # Extract coordinates (x1, y1, x2, y2) and confidence
    x1, y1, x2, y2 = box.xyxy[0]  # Get the top-left and bottom-right coordinates
    confidence = box.conf[0]      # Get the confidence score
    
    # Print the values
    print(f"Box Coordinates: ({x1:.2f}, {y1:.2f}, {x2:.2f}, {y2:.2f})")
    print(f"Confidence: {confidence:.2f}")
