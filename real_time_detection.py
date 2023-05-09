# import torch
# import cv2
# import os
# import numpy as np


# # Load YOLOv5 model
# #model = torch.load('./yolov5.pt')
# #model = torch.hub.load('./yolov5.pt', 'custom', path='./yolov5.pt', source='local')
# model = torch.hub.load('../yolov5', 'custom', path='./runs/train/exp/weights/best.pt', source='local')
# #model = torch.hub.load('../yolov5', 'custom', path='./yolov5_5k.pt', source='local')

# model_name='yolov5.pt'
# #model = torch.hub.load(os.getcwd(), 'custom', source='local', path = model_name, force_reload = True)


# # Open the camera
# cap = cv2.VideoCapture(0)

# while True:
#     # Read a frame from the camera
#     ret, frame = cap.read()
    
#     # Perform object detection on the frame
#     results = model(frame)
    
#     # Display the frame with the detected objects
#     cv2.imshow('Object detection', np.squeeze(results.render()))
    
#     # Exit if the 'q' key is pressed
#     if cv2.waitKey(1) == ord('q'):
#         break

# # Release the camera and close the window
# cap.release()
# cv2.destroyAllWindows()



import cv2
import numpy as np
import pyrealsense2 as rs
import torch

# Load YOLOv5 model
model = torch.hub.load('../yolov5', 'custom', path='./runs/train/exp/weights/best.pt', source='local')
model_name='yolov5.pt'

# Open the Realsense pipeline and configure the color stream
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

while True:
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    # Convert the frame to a numpy array
    frame = np.asanyarray(color_frame.get_data())

    # Perform object detection on the frame
    results = model(frame)

    # Display the frame with the detected objects
    cv2.imshow('Object detection', np.squeeze(results.render()))

    # Exit if the 'q' key is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# Release the Realsense pipeline and close the window
pipeline.stop()
cv2.destroyAllWindows()
