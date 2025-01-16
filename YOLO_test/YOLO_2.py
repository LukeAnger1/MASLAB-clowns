from ultralytics import YOLO
import cv2

# Load a YOLO model (pre-trained YOLOv8 nano model)
model = YOLO('yolov8n.pt')  # Replace with your model if needed

# Load an image
image_path = "image.png"  # Replace with your image path
image = cv2.imread(image_path)

# Run object detection
results = model(image)

# Iterate over the results
for result in results:
    boxes = result.boxes      # Access bounding box outputs
    masks = result.masks      # Access segmentation masks (if applicable)
    keypoints = result.keypoints  # Access keypoints (if applicable)
    probs = result.probs      # Access classification probabilities (if applicable)
    obb = result.obb          # Access oriented bounding boxes (if applicable)

    # Display or save the results
    result.show()             # Display the image with annotations
    result.save("output.jpg") # Save the annotated image

# Extract the first result (since we're working with one image)
result = results[0]

# Iterate through the bounding boxes
for box in result.boxes:
    # Extract coordinates (x1, y1, x2, y2) and confidence
    x1, y1, x2, y2 = map(int, box.xyxy[0]) # box.xyxy[0]  # Get the top-left and bottom-right coordinates
    confidence = box.conf[0]      # Get the confidence score

    # Print the values
    print(f"Box Coordinates: ({x1:.2f}, {y1:.2f}, {x2:.2f}, {y2:.2f})")
    print(f"Confidence: {confidence:.2f}")
