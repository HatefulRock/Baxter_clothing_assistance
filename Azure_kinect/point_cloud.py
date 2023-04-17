import numpy as np
import pyk4a
from pyk4a import Config, PyK4A
from pyk4a import PyK4APlayback
import torch
import cv2


def extract_jacket_point_cloud(k4a: PyK4A, image: np.ndarray, bbox: list, model: torch.nn.Module):
    """
    Extracts the point cloud of the jacket within the specified bounding box.

    Args:
        k4a (PyK4A): An instance of the PyK4A class.
        image (np.ndarray): The color image of the current frame.
        bbox (list): A list containing the bounding box coordinates in [xmin, ymin, xmax, ymax] format.
        model (torch.nn.Module): The YOLOv5 model used for detecting the bounding box.

    Returns:
        np.ndarray: A numpy array containing the 3D point cloud of the jacket.
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
        roi_bbox = pred_bbox[bbox]
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

        return points

    finally:
        # Stop the camera when done.
        k4a.stop_devices()


# Example usage:
k4a = PyK4A()
image = cv2.imread('path/to/image.jpg')  # Replace with the actual image path.
