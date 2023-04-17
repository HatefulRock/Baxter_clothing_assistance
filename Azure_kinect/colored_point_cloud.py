import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
import pyk4a
from pyk4a import Config, PyK4A
from pyk4a import PyK4APlayback
import numpy as np
import torch
import cv2
import std_msgs.msg


def extract_jacket_point_cloud(k4a: PyK4A, image: np.ndarray, model: torch.nn.Module):
    """
    Extracts the point cloud of the jacket within the specified bounding box.

    Args:
        k4a (PyK4A): An instance of the PyK4A class.
        image (np.ndarray): The color image of the current frame.
        model (torch.nn.Module): The YOLOv5 model used for detecting the bounding box.

    Returns:
        tuple: A tuple containing the 3D point cloud of the jacket and the 2D pixel coordinates of the gripping points.
    """

    # Configure the camera to capture the color, depth and point cloud data.
    config = Config(
        color_resolution=pyk4a.ColorResolution.RES_1080P,
        depth_mode=pyk4a.DepthMode.WFOV_2X2BINNED,
        synchronized_images_only=True,
        enable_point_cloud=True,
    )

    # Start the camera with the configured parameters.
    k4a.start_devices(config)

    # Initialize the ROS node and publishers.
    rospy.init_node('jacket_grabber')
    pc_pub = rospy.Publisher('/jacket_point_cloud', PointCloud2, queue_size=10)
    gp_pub = rospy.Publisher('/gripping_points', Point, queue_size=10)

    try:
        # Resize the color image to the input size of the YOLOv5 model.
        input_size = (640, 640)  # Replace with the actual input size of the YOLOv5 model.
        resized_image = cv2.resize(image, input_size)

        # Convert the resized image to a PyTorch tensor.
        tensor_image = torch.from_numpy(resized_image).permute(2, 0, 1).unsqueeze(0).float() / 255.0

        # Pass the tensor image through the YOLOv5 model to get the predicted bounding box.
        pred = model(tensor_image)[0]
        pred_bbox = pred[:, :4].detach().cpu().numpy()

        # Extract the region of interest (ROI) of the jacket.
        roi_bbox = pred_bbox[0]  # Assuming the jacket is the first object detected.
        xmin, ymin, xmax, ymax = roi_bbox.astype(int)
        roi_image = image[ymin:ymax, xmin:xmax]

        # Extract the color and point cloud data from the capture.
        capture = k4a.get_capture()
        color_image = capture.color
        point_cloud = capture.transformed_depth_point_cloud

        # Extract the point cloud of the ROI of the jacket.
        roi_point_cloud = point_cloud[ymin:ymax, xmin:xmax]

        # Filter out points that are outside the range of the jacket.
        mask = roi_point_cloud[:, :, 2] < np.mean(roi_point_cloud[:, :, 2]) + np.std(roi_point_cloud[:, :, 2])
        roi_point_cloud[mask] = np.nan

        # Flatten the point cloud into a 2D array.
        points = roi_point_cloud.reshape(-1, 3)

        # Remove any NaN values from the flattened point cloud.
        points = points[~np.isnan(points).any(axis=1)]

        # Create the header for the PointCloud2 message.
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'jacket_frame'

        # Define the fields for the PointCloud2 message.
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        # Create the PointCloud2 message from the flattened point cloud.
        point_cloud_msg = pc2.create_cloud(header, fields, points)

        # Publish the PointCloud2 message.
        pc_pub.publish(point_cloud_msg)

        # Compute the 2D pixel coordinates of the gripping points using the color image.
        left_grip_point = (xmin, ymin)  # Replace with the actual pixel coordinates of the left gripping point.
        right_grip_point = (xmax, ymin)  # Replace with the actual pixel coordinates of the right gripping point.

        # Publish the gripping points as ROS messages.
        left_grip_msg = Point(x=left_grip_point[0], y=left_grip_point[1])
        right_grip_msg = Point(x=right_grip_point[0], y=right_grip_point[1])
        gp_pub.publish(left_grip_msg)
        gp_pub.publish(right_grip_msg)

    finally:
        # Stop the camera.
        k4a.stop()

"""
In this modified code, we first initialize two ROS publishers for publishing the 3D point cloud and the gripping points. We then create a PointCloud2 message from the flattened point cloud, and publish it using the pc_pub publisher.

We also compute the 2D pixel coordinates of the gripping points using the color image, and publish them as ROS messages using the gp_pub publisher.

You can then subscribe to these topics in your Baxter control code to retrieve the 3D point cloud and gripping points, and use them to control the robot's arm.

"""