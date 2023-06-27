import cv2
import numpy as np
import math
import pyrealsense2 as rs
import matplotlib.pyplot as plt




def isolate_green_tape(image):
    # Convert the image from BGR to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper boundaries of the green color in HSV
    # lower_green = np.array([40, 40, 40])  # Adjust these values as per your requirements
    # upper_green = np.array([70, 255, 255])

    lower_green = np.array([24, 100, 100])  # Adjust these values as per your requirements
    upper_green = np.array([30, 255, 255])

    # Create a mask based on the green color range
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # Apply the mask to the original image
    isolated_tape = cv2.bitwise_and(image, image, mask=mask)

    return isolated_tape

def find_cluster_midpoints(mask_image, num_clusters=2):
    # Load the mask image
    #mask = cv2.imread(mask_image_path)

    # Convert the image to grayscale
    gray = cv2.cvtColor(mask_image, cv2.COLOR_BGR2GRAY)

    #print("gray",gray)

    # Apply threshold to create a binary image
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)

    #print("binary",binary)

    #visualize the binary image
    #plt.imshow(binary)
    #plt.show()



    # Perform connected component labeling
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary)
    # print("num_labels",num_labels)
    # print("labels",labels)
    # print("stats",stats)
    # print("centroids",centroids)

    # Exclude the background label (index 0)
    labels = labels[1:]
    stats = stats[1:]
    centroids = centroids[1:]

    # # Sort the clusters by area (descending order)
    sorted_indices = np.argsort(stats[:, cv2.CC_STAT_AREA])[::-1]

    # # Keep only the specified number of largest clusters
    sorted_indices = sorted_indices[:num_clusters]

    # Get the midpoints of the largest clusters
    midpoints = centroids[sorted_indices].astype(int)
    #midpoints = centroids.astype(int)

    return midpoints.tolist()


def get_3d_coordinates(midpoints, depth_frame, depth_intrinsics):
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
    frames = pipeline.wait_for_frames()
    # Align depth frame to color frame
    aligned_frames = align.process(frames)

    # Get aligned frames
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    
    #depth_frame = decimate.process(depth_frame)
    
    # Grab new intrinsics (may be changed by decimation)
    depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
    w, h = depth_intrinsics.width, depth_intrinsics.height

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    #plt.imshow(depth_colormap)
    #plt.show()

    # Create alignment primitive with color as its target stream:
    align = rs.align(rs.stream.color)
    frameset = align.process(frames)

    # Update color and depth frames:
    aligned_depth_frame = frameset.get_depth_frame()
    aligned_depth_image = np.asanyarray(aligned_depth_frame.get_data())
    colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())


    # Isolate the green tape
    isolated_tape = isolate_green_tape(color_image)

    mapped_frame, color_source = aligned_depth_frame, colorized_depth

    points = pc.calculate(aligned_depth_frame)
    pc.map_to(mapped_frame)

    # Pointcloud data to arrays
    v, t = points.get_vertices(), points.get_texture_coordinates()
    verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
    texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

    point_cloud_np = np.ascontiguousarray(verts).view(np.float32).reshape(-1, 3)
    #print("point cloud np",point_cloud_np)

    # Generate the mask for the green tape
    green_mask = cv2.inRange(isolated_tape, np.array([1, 1, 1]), np.array([255, 255, 255]))


    #cv2.imshow("depth image", depth_image)
    #cv2.imshow("aligned depth image", aligned_depth_image)
    #cv2.imshow("Green Mask", green_mask)
    #cv2.imshow("colorized depth", colorized_depth)
    #cv2.imshow("depth_colormap", depth_colormap)

    #Print the gripping points
    #print("Gripping Points (x, y, z):")
    #for point in gripping_points:
        #print(f"({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})")

    
    # Find the clusters in the green mask
    midpoints = find_cluster_midpoints(isolated_tape)
    print("Midpoints:", midpoints)

    # Get the 3D coordinates of the midpoints
    midpoints_3d = get_3d_coordinates(midpoints, depth_image, depth_intrinsics)
    print("Midpoints 3D:", midpoints_3d)



    # Visualize the midpoints on the image
    for midpoint in midpoints:
        x, y = midpoint
        cv2.circle(isolated_tape, (x, y), 5, (0, 255, 0), -1)

    # Display the original image and the isolated green tape
    cv2.imshow("Original Image", color_image)
    cv2.imshow("Isolated Green Tape", isolated_tape)
    # Exit the program when the user presses 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the pipeline
pipeline.stop()

# Close all windows
cv2.destroyAllWindows()
