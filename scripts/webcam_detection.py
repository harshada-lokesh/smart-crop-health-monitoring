from ultralytics import YOLO
import cv2
import numpy as np

# Load your trained YOLOv11 model
model = YOLO("C:/Users/harsh/Downloads/my_model.pt")

# Define solutions dictionary (add all your classes as needed)
solutions = {
    "Bell_pepper__leaf_healthy": "✅ Your bell pepper plant is healthy! Maintain good care.",
    "Bell_pepper__leaf_spot": "⚠️ Leaf spot detected! Remove affected leaves and use a copper fungicide.",
    "Corn_gray_leaf_spot": "⚠️ Gray leaf spot detected! Use resistant hybrids and apply fungicides.",
    "Corn_leaf_blight": "⚠️ Leaf blight detected! Rotate crops and apply appropriate fungicides.",
    "Corn_leaf_healthy": "✅ Your corn plant is healthy! Keep monitoring it.",
    "Corn_rust_leaf": "⚠️ Rust detected! Use resistant varieties and apply fungicides.",
    "Grape_leaf_black_rot": "⚠️ Black rot detected! Prune infected areas and apply fungicides.",
    "Grape_leaf_healthy": "✅ Your grape plant is healthy! Keep up with good care.",
    "Potato_leaf_early_blight": "⚠️ Early blight detected! Remove infected leaves and apply fungicide.",
    "Potato_leaf_healthy": "✅ Your potato plant is healthy! Keep monitoring its growth.",
    "Potato_leaf_late_blight": "⚠️ Late blight detected! Remove infected plants and apply fungicide.",
    "Strawberry_leaf_healthy": "✅ Your strawberry plant is healthy! Maintain proper watering.",
    "Strawberry_leaf_scrorch": "⚠️ Leaf scorch detected! Increase watering and provide shade.",
    "Tomato_leaf_bacterial_spot": "⚠️ Bacterial spot detected! Remove affected leaves and avoid overhead watering.",
    "Tomato_leaf_early_blight": "⚠️ Early blight detected! Apply fungicide and rotate crops.",
    "Tomato_leaf_healthy": "✅ Your tomato plant is healthy! Keep an eye on it.",
    "Tomato_leaf_late_blight": "⚠️ Late blight detected! Remove affected plants and use fungicides.",
    "Tomato_leaf_mold": "⚠️ Mold detected! Improve air circulation and reduce humidity.",
    "Tomato_leaf_mosaic_virus": "⚠️ Mosaic virus detected! Remove infected plants and control insects.",
    "Tomato_leaf_septoria_spot": "⚠️ Septoria spot detected! Apply fungicide and practice good crop rotation.",
    "Tomato_leaf_yellow_virus": "⚠️ Yellow virus detected! Control insects and remove infected plants."
}

def process_frame(frame):
    # Run YOLO detection on the frame
    results = model(frame)
    
    # Loop through detections
    for result in results:
        boxes = result.boxes.xyxy  # Bounding box coordinates
        scores = result.boxes.conf  # Confidence scores
        class_ids = result.boxes.cls  # Class IDs
        class_names = result.names  # Class names

        for box, score, class_id in zip(boxes, scores, class_ids):
            class_id = int(class_id)
            label = class_names[class_id]
            solution = solutions.get(label, "No solution available")

            # Draw bounding box
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Red box

            # Add label and solution text
            text = f"{label} ({score:.2f})"
            sol_text = f"{solution}"
            cv2.putText(frame, text, (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, sol_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    return frame

# Start webcam capture (0 is the default camera, use 1 or 2 if you have multiple cameras)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Webcam started. Press 'q' to quit.")
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Process the frame with YOLO and display results
    annotated_frame = process_frame(frame)
    cv2.imshow("Plant Disease Detection", annotated_frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
cap.release()
cv2.destroyAllWindows()