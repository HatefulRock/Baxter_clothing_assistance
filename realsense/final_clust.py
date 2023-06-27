import cv2
import numpy as np
import math
import pyrealsense2 as rs
import matplotlib.pyplot as plt
import socket

# Define the server address and port
server_address = '192.168.0.21'
server_port = 5353

# Create a socket object
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the server
sock.connect((server_address, server_port))

def isolate_green_tape(image):
    try:
        # Convert the image from BGR to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper boundaries of the green color in HSV
        lower_green = np.array([24, 100, 100])  # Adjust these values as per your requirements
        upper_green = np.array([30, 255, 255])

        # Create a mask based on the green color range
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Apply the mask to the original image
        isolated_tape = cv2.bitwise_and(image, image, mask=mask)

        return isolated_tape
    except Exception as e:
        print("Error in isolate_green_tape:", str(e))
        return None


def find_cluster_midpoints(mask_image, num_clusters=2):
    try:
        # Convert the image to grayscale
        gray = cv2.cvtColor(mask_image, cv2.COLOR_BGR2GRAY)

        # Apply threshold to create a binary image
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)

        # Perform connected component labeling
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary)

        # Exclude the background label (index 0)
        labels = labels[1:]
        stats = stats[1:]
        centroids = centroids[1:]

        # Sort the clusters by area (descending order)
        sorted_indices = np.argsort(stats[:, cv2.CC_STAT_AREA])[::-1]

        # Keep only the specified number of largest clusters
        sorted_indices = sorted_indices[:num_clusters]

        # Get the midpoints of the largest clusters
        midpoints = centroids[sorted_indices].astype(int)

        return midpoints.tolist()
    except Exception as e:
        print("Error in find_cluster_midpoints:", str(e))
        return []


def get_3d_coordinates(midpoints, depth_frame, depth_intrinsics):
    try:
        # Convert midpoints to NumPy array
        midpoints = np.array(midpoints)

        # Convert midpoints to homogeneous coordinates
        midpoints_homogeneous = np.hstack((midpoints, np.ones((midpoints.shape[0], 1))))

        # Get the depth values at the midpoints' image coordinates
        depth_values = depth_frame[midpoints[:, 1], midpoints[:, 0]]

        # Create the intrinsics matrix
        intrinsics_matrix = np.array([[depth_intrinsics.fx, 0, depth_intrinsics.ppx],
                                      [0, depth_intrinsics.fy, depth_intrinsics.ppy],
                                      [0, 0, 1]])

        # Compute the inverse of the intrinsics matrix
        intrinsics_inv = np.linalg.inv(intrinsics_matrix)

        # Calculate the 3D coordinates of the midpoints
        midpoints_3d = depth_values[:, np.newaxis] * np.dot(intrinsics_inv, midpoints_homogeneous.T).T

        # Convert coordinates from millimeters to meters
        midpoints_3d /= 1000.0

        return midpoints_3d
    except Exception as e:
        print("Error in get_3d_coordinates:", str(e))
        return []


# Initialize the Intel RealSense pipeline
pipeline = rs.pipeline()

# Configure the pipeline
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

align = rs.align(rs.stream.color)

# Start the pipeline
pipeline.start(config)
colorizer = rs.colorizer()

# Get stream profile and camera intrinsics
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

pc = rs.pointcloud()

while True:
    try:
        frames = pipeline.wait_for_frames()
        # Align depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Isolate the green tape
        isolated_tape = isolate_green_tape(color_image)

        # Find the clusters in the green mask
        midpoints = find_cluster_midpoints(isolated_tape)

        # Get the 3D coordinates of the midpoints
        midpoints_3d = get_3d_coordinates(midpoints, depth_image, depth_intrinsics)

        # Visualize the midpoints on the image
        for midpoint in midpoints:
            x, y = midpoint
            cv2.circle(isolated_tape, (x, y), 5, (0, 255, 0), -1)

        # Display the original image and the isolated green tape
        cv2.imshow("Original Image", color_image)
        cv2.imshow("Isolated Green Tape", isolated_tape)

        # Extract the midpoints from midpoints_3d
        midpoints = midpoints_3d[:, :3]

        # Convert the midpoints to a string representation
        midpoints_str = np.array2string(midpoints, separator=',')[1:-1]

        # Send the midpoints to the server
        sock.sendall(midpoints_str.encode())

        # Exit the program when the user presses 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print("Error:", str(e))

# Stop the pipeline
pipeline.stop()

#close socket
sock.close()

# Close all windows
cv2.destroyAllWindows()
