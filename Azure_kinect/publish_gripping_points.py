import rospy
import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
from sklearn.cluster import DBSCAN
from baxter_interface import Limb, gripper as robot_gripper

# Initialize the Baxter robot
rospy.init_node('gripping_node')
limb = Limb('right')
gripper = robot_gripper.Gripper('right')

# Open the robot's gripper
gripper.calibrate()
gripper.open()

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

# Find the contours in the mask
contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Loop over the contours and find the gripping points
for contour in contours:
    moments = cv2.moments(contour)
    if moments['m00'] > 0:
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        depth = depth_image[cy, cx]

    # Convert the gripping point to a 3D point in camera coordinates
    point = rs.rs2_deproject_pixel_to_point(intrinsics, [cx], [cy], depth)
    point = np.asarray(point)

    # Transform the 3D point from camera coordinates to robot base coordinates
    trans = np.array([0.075, 0.34, 0.135])
    rot = np.array([[-0.49976452, -0.49972557, 0.70663573],
                    [-0.49972557, 0.50023543, 0.70659575],
                    [-0.70663573, -0.70659575, -0.0176623 ]])
    point = np.dot(rot, point) + trans

    # Move the robot arm to the gripping point
    limb.move_to_neutral()
    limb.set_joint_positions({
        'right_s0': -0.04537729460538968,
        'right_s1': -0.7592122490127984,
        'right_e0': 0.01919862112984956,
        'right_e1': 1.5838728303618693,
        'right_w0': -0.01919862112984956,
        'right_w1': 1.482614309437932,
        'right_w2': -0.03237262656649555
    })
    limb.move_to_joint_positions({
        'right_s0': 0.15581199269851357,
        'right_s1': -0.9339021483121755,
        'right_e0': 0.284434929422089,
        'right_e1': 2.102425326108291,
        'right_w0': -0.35587816433064105,
        'right_w1': 1.6839558404296698,
        'right_w2': -0.3001784621465488
    })

    # Close the robot gripper to grab the object
    gripper.close()

    # Move the robot arm up to lift the object
    limb.move_to_joint_positions({
        'right_s0': 0.4339761533215537,
        'right_s1': -0.6011947733693227,
        'right_e0': -0.33929248542766013,
        'right_e1': 1.7120324809787172,
        'right_w0': 0.021027383996580394,
        'right_w1': 1.301444384487938,
        'right_w2': -0.0030425260488997693
    })

    # Move the robot arm to a neutral position
    limb.move_to_neutral()
