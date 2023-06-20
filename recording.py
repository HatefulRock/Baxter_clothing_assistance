import torch
import cv2
import os
import numpy as np


# Load YOLOv5 model
#model = torch.load('./yolov5.pt')
#model = torch.hub.load('./yolov5.pt', 'custom', path='./yolov5.pt', source='local')
model = torch.hub.load('../yolov5', 'custom', path='./runs/train/exp/weights/best.pt', source='local')
#model = torch.hub.load('../yolov5', 'custom', path='./yolov5_5k.pt', source='local')

model_name='yolov5.pt'
#model = torch.hub.load(os.getcwd(), 'custom', source='local', path = model_name, force_reload = True)
#save the video
# Save the video
fourcc = cv2.VideoWriter_fourcc(*'XVID')
output_file = 'output.avi'
frame_rate = 30.0
frame_size = (640, 480)
video_writer = cv2.VideoWriter(output_file, fourcc, frame_rate, frame_size)

# Open the camera
cap = cv2.VideoCapture(0)

while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    # Perform object detection on the frame
    results = model(frame)

    # Process the detected objects
    for result in results.xyxy[0]:
        # Extract bounding box coordinates and class label
        x1, y1, x2, y2, conf, cls = result.tolist()

        # Draw bounding box and label on the frame
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(frame, f'{int(cls)}: {conf:.2f}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Write the frame to the video file
    video_writer.write(frame)

    # Display the frame with the detected objects
    cv2.imshow('Object detection', frame)

    # Exit if the 'q' key is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# Release the camera and video writer
cap.release()
video_writer.release()

# Close all OpenCV windows
cv2.destroyAllWindows()
