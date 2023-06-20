import cv2
import numpy as np
import pyrealsense2 as rs

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start pipeline
pipeline.start(config)

while True:
    # Wait for the next frame
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    if not depth_frame:
        continue

    # Convert depth frame to numpy array
    depth_image = np.asanyarray(depth_frame.get_data())

    # Apply colormap to depth image for visualization
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    # Display depth image
    cv2.imshow('Depth Image', depth_colormap)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
pipeline.stop()
cv2.destroyAllWindows()
