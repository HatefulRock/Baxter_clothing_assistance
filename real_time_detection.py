import torch
import cv2
import os
import numpy as np


# Load YOLOv5 model
#model = torch.load('./yolov5.pt')
#model = torch.hub.load('./yolov5.pt', 'custom', path='./yolov5.pt', source='local')
model = torch.hub.load('../yolov5', 'custom', path='./best.pt', source='local')
model_name='yolov5.pt'
#model = torch.hub.load(os.getcwd(), 'custom', source='local', path = model_name, force_reload = True)


# Open the camera
cap = cv2.VideoCapture(0)

while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    
    # Perform object detection on the frame
    results = model(frame)
    
    # Display the frame with the detected objects
    cv2.imshow('Object detection', np.squeeze(results.render()))
    
    # Exit if the 'q' key is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
