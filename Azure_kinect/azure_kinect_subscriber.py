import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs

# Initialize the camera pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.rgb8, 30)
config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
pipeline.start(config)

# Wait for the camera to stabilize
for i in range(30):
    pipeline.wait_for_frames()

# Capture a single frame from the camera
frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()
depth_frame = frames.get_depth_frame()

# Convert the frames to numpy arrays
color_image = np.asarray(color_frame.get_data())
depth_image = np.asarray(depth_frame.get_data())

# Convert the color image to point cloud data
intrinsics = rs.video_stream_profile(color_frame.profile).get_intrinsics()
points = rs.rs2_deproject_pixel_to_point(intrinsics, [x for y in range(depth_image.shape[0]) for x in range(depth_image.shape[1])], \
                                          [y for y in range(depth_image.shape[0]) for x in range(depth_image.shape[1])], \
                                          depth_image.ravel())
points = np.reshape(points, (depth_image.shape[0], depth_image.shape[1], 3))

# Convert the point cloud data to an Open3D point cloud object
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points.reshape(-1, 3))
pcd.colors = o3d.utility.Vector3dVector(color_image.reshape(-1, 3) / 255.0)

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])
