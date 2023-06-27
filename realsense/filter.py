import cv2
import numpy as np
import math
import pyrealsense2 as rs
import matplotlib.pyplot as plt


class AppState:

    def __init__(self, *args, **kwargs):
        self.WIN_NAME = 'RealSense'
        self.pitch, self.yaw = math.radians(-10), math.radians(-15)
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = True

    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation[:] = 0, 0, -1

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)


state = AppState()
# def isolate_green_tape(image_path):
#     # Load the image
#     image = cv2.imread(image_path)

#     # Convert the image from BGR to HSV color space
#     hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

#     # Define the lower and upper boundaries of the green color in HSV
#     lower_green = np.array([40, 40, 40])  # Adjust these values as per your requirements
#     upper_green = np.array([70, 255, 255])

#     # Create a mask based on the green color range
#     mask = cv2.inRange(hsv, lower_green, upper_green)

#     # Apply the mask to the original image
#     isolated_tape = cv2.bitwise_and(image, image, mask=mask)

#     #resize the images to fit the screen
#     scale_percent = 30 #percent of original size
#     width = int(mask.shape[1] * scale_percent / 100)
#     height = int(mask.shape[0] * scale_percent / 100)
#     dim = (width, height)
#     mask = cv2.resize(mask, dim, interpolation = cv2.INTER_AREA)
#     image = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
#     isolated_tape = cv2.resize(isolated_tape, dim, interpolation = cv2.INTER_AREA)
    

#     # Display the original image and the isolated green tape
#     cv2.imshow("Original Image", image)
#     cv2.imshow("Isolated Green Tape", isolated_tape)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

# Provide the path to your image file
# image_path = "pic.jpg"
# isolate_green_tape(image_path)

#use the isoate_green_tape function with an intel realsense camera

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

def get_gripping_points(point_cloud, mask):

    #print("point cloud",point_cloud)
    #print("mask",mask)
    #check if mask is empty
   # if mask is None:
        #print("mask is empty")
    #else:
        #print("mask is not empty")
    # Flatten the point cloud data and mask for processing
    point_cloud_flat = point_cloud.reshape(-1, 3)
    mask_flat = mask.flatten()

    #Filter points using the mask
    #print("mask shape",mask_flat.shape)
    #print("point cloud shape",point_cloud_flat.shape)
    #print("mask",mask_flat)
    #print("point cloud",point_cloud_flat)
    points_filtered = point_cloud_flat[mask_flat]
    print("points filtered",points_filtered)

    #check if points_filtered is empty
    # if points_filtered is None:
    #     print("points_filtered is empty")
    # else:
    #     #print the points that are not zero
    #     print("points_filtered is not empty")
    #     # for i in range(len(points_filtered)):
    #     #     if points_filtered[i][0] != 0:
    #     #         print(points_filtered[i])

    # Calculate the centroid of the filtered points
    centroid = np.mean(points_filtered, axis=0)
    # if centroid is None:
    #     print("centroid is empty")
    # else:
    #     print("centroid is not empty")
    #     print("centroid",centroid.shape)
    #     print("centroid",centroid)


    # Sort the points based on their distance from the centroid
    sorted_points = points_filtered[np.argsort(np.linalg.norm(points_filtered - centroid, axis=1))]
    #print("sorted points",sorted_points)

    # Take the first two closest points as the gripping points
    gripping_points = sorted_points[:2]

    return gripping_points

# Initialize the Intel RealSense pipeline
pipeline = rs.pipeline()

# Configure the pipeline
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

align = rs.align(rs.stream.color)


# Start the pipeline
pipeline.start(config)
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
colorizer = rs.colorizer()

# Get stream profile and camera intrinsics
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

pc = rs.pointcloud()
while True:
    # Grab camera data
    if not state.paused:
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

        # if state.color:
        #     mapped_frame, color_source = color_frame, color_image
        # else:
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

        # Get the gripping points corresponding to the green tape
        gripping_points = get_gripping_points(point_cloud_np, green_mask)

        # Display the original image and the isolated green tape
        cv2.imshow("Original Image", color_image)
        cv2.imshow("Isolated Green Tape", isolated_tape)
        #cv2.imshow("depth image", depth_image)
        #cv2.imshow("aligned depth image", aligned_depth_image)
        #cv2.imshow("Green Mask", green_mask)
        #cv2.imshow("colorized depth", colorized_depth)
        #cv2.imshow("depth_colormap", depth_colormap)

        #Print the gripping points
        #print("Gripping Points (x, y, z):")
        #for point in gripping_points:
            #print(f"({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})")

        # Exit the program when the user presses 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Stop the pipeline
pipeline.stop()

# Close all windows
cv2.destroyAllWindows()

