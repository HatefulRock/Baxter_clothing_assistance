import cv2
import numpy as np
import pyrealsense2 as rs

# Load OpenPose model
net = cv2.dnn.readNetFromCaffe('pose_deploy.prototxt')

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start pipeline
pipeline.start(config)

while True:
    # Wait for the next frame
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue

    # Convert frame to numpy array
    color_image = np.asanyarray(color_frame.get_data())

    # Perform skeleton tracking
    blob = cv2.dnn.blobFromImage(color_image, 1.0, (368, 368), (127.5, 127.5, 127.5), swapRB=True, crop=False)
    net.setInput(blob)
    output = net.forward()

    # Extract keypoints
    keypoints = []
    for i in range(output.shape[0]):
        prob_map = output[i, :, :, :]
        prob_map = cv2.resize(prob_map, (color_image.shape[1], color_image.shape[0]))
        keypoints_map = np.argmax(prob_map, axis=0)
        keypoints.append(keypoints_map)

    # Display skeleton
    cv2.imshow('Skeleton Tracking', color_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
pipeline.stop()
cv2.destroyAllWindows()
