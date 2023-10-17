import numpy as np

def find_transformation(left_realsense_points, baxter_points):
    """
    Find the transformation matrix that maps points from the left Realsense camera system to the Baxter system.

    Args:
        left_realsense_points (ndarray): Array of points in the left Realsense camera system.
        baxter_points (ndarray): Array of points in the Baxter system.

    Returns:
        ndarray: The transformation matrix that maps points from the left Realsense camera system to the Baxter system.
    """
    A = np.hstack((left_realsense_points, np.ones((left_realsense_points.shape[0], 1))))
    B = np.hstack((baxter_points, np.ones((baxter_points.shape[0], 1))))

    T, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
    return T

def transform_points(points, transformation_matrix):
    """
    Transform points from the left Realsense camera system to the Baxter system using the provided transformation matrix.

    Args:
        points (ndarray): Array of points in the left Realsense camera system.
        transformation_matrix (ndarray): The transformation matrix that maps points from the left Realsense camera system
                                         to the Baxter system.

    Returns:
        ndarray: The transformed points in the Baxter system.
    """
    augmented_points = np.hstack((points, np.ones((points.shape[0], 1))))
    transformed_points = np.dot(augmented_points, transformation_matrix.T)
    return transformed_points[:, :3]


# Example usage
left_realsense_points = np.array([[0.593, 0.2845, 0.1726],
                                  [0.572, -0.1335, 0.2121]])

baxter_points = np.array([[0.8078, 0.1432, 0.0949],
                          [0.8384, -0.1338, 0.1291]])

transformation_matrix = find_transformation(left_realsense_points, baxter_points)
print("Transformation matrix:")
print(transformation_matrix)

new_left_realsense_points = np.array([[0.587, 0.160, 0.252],
                                      [0.580, 0.211, 0.158]])



transformed_points = transform_points(new_left_realsense_points, transformation_matrix)
print("Transformed points:")
print(transformed_points)
