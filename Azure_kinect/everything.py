import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
from sklearn.cluster import DBSCAN

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

# Perform DBSCAN clustering on the point cloud data
clustering = DBSCAN(eps=0.03, min_samples=10).fit(pcd.points)
labels = clustering.labels_

# Create a mask for the clustered points
mask = np.zeros(depth_image.shape, dtype=np.uint8)
for i in range(labels.max() + 1):
    cluster_mask = (labels == i)
    if np.sum(cluster_mask) > 0:
        cluster_points = pcd.points[np.where(cluster_mask)[0], :]
        bounding_box = cv2.boundingRect(np.array(cluster_points[:, :2], dtype=np.int32))
        cv2.rectangle(mask, (bounding_box[0], bounding_box[1]), (bounding_box[0] + bounding_box[2], bounding_box[1] + bounding_box[3]), (255,), -1)

# Apply the mask to the color image
result_image = cv2.bitwise_and(color_image, color_image, mask=mask)

# Convert the result image to grayscale
gray_image = cv2.cvtColor(result_image, cv2.COLOR_BGR2GRAY)

# Apply a threshold to the grayscale image
_, thresh_image = cv2.threshold(gray_image, 10, 255, cv2.THRESH_BINARY)

# Find contours in the threshold image
contours, _ = cv2.findContours(thresh_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Draw the contours on the original color image
for contour in contours:
    if cv2.contourArea(contour) > 50:
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(color_image, [box], 0, (0, 255, 0), 2)

# Show the resulting image
cv2.imshow('Result', color_image)
cv2.waitKey(0)

# Determine the coordinates of the gripping points
gripping_points = []
for contour in contours:
    if cv2.contourArea(contour) > 50:
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        gripping_point = np.mean(box, axis=0)
        gripping_points.append(gripping_point)

# Convert the gripping points to 3D coordinates
gripping_points_3d = []
for gripping_point in gripping_points:
    x = int(gripping_point[0])
    y = int(gripping_point[1])
    gripping_points_3d.append(points[y, x])

# Stop the camera pipeline
pipeline.stop()

# Send the gripping points to the robot
# Code to send gripping points to the robot goes here
import rospy
from geometry_msgs.msg import PointStamped

# Initialize ROS node
rospy.init_node('gripping_point_publisher')

# Define the topic to publish the gripping points to
topic = '/gripping_points'

# Define the publisher
pub = rospy.Publisher(topic, PointStamped, queue_size=10)

# Loop over the gripping points and publish each one
for gripping_point_3d in gripping_points_3d:
    # Create a PointStamped message
    gripping_point_msg = PointStamped()

    # Fill in the message fields
    gripping_point_msg.header.stamp = rospy.Time.now()
    gripping_point_msg.header.frame_id = 'camera_link'
    gripping_point_msg.point.x = gripping_point_3d[0]
    gripping_point_msg.point.y = gripping_point_3d[1]
    gripping_point_msg.point.z = gripping_point_3d[2]

    # Publish the message
    pub.publish(gripping_point_msg)

# Spin once to publish the messages
rospy.spin()
