import cv2
import mediapipe as mp
import pyrealsense2 as rs
import numpy as np



def vect_func(x1,x2,y1,y2):
    return (x2-x1,y2-y1)







# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Create alignment object
align = rs.align(rs.stream.color)

# Start the pipeline
pipeline.start(config)

# Create a pose estimation instance
mp_pose = mp.solutions.pose

# Initialize the pose estimator
pose_estimator = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Process the video stream
while True:
    # Wait for a new frame from the RealSense camera
    frames = pipeline.wait_for_frames()

    # Align depth frame to color frame
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    if not color_frame:
        continue

    # Convert the color frame to a numpy array
    image = np.asanyarray(color_frame.get_data())

    # Process the frame
    results = pose_estimator.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

    # Extract the landmark coordinates and numbers
    if results.pose_landmarks is not None:
        # Get the image height and width
        image_height, image_width, _ = image.shape

        # Loop through each detected landmark
        for index, landmark in enumerate(results.pose_landmarks.landmark):
            # Extract the X and Y coordinates of the landmark
            x = int(landmark.x * image_width)
            y = int(landmark.y * image_height)

            # Draw a circle at the landmark position
            cv2.circle(image, (x, y), 5, (0, 255, 0), -1)

            # Put the landmark number beside the circle
            cv2.putText(image, str(index), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

        # Define the landmark indices for the left and right arms
        left_landmark_indices = [11, 13, 15]  # Left shoulder, left elbow, left wrist
        right_landmark_indices = [12, 14, 16]  # Right shoulder, right elbow, right wrist

        # Extract the coordinates of left landmarks
        left_landmarks = []
        for index in left_landmark_indices:
            if results.pose_landmarks and results.pose_landmarks.landmark:
                landmark = results.pose_landmarks.landmark[index]
                left_landmarks.append((int(landmark.x * image_width), int(landmark.y * image_height)))

        # Extract the coordinates of right landmarks
        right_landmarks = []
        for index in right_landmark_indices:
            if results.pose_landmarks and results.pose_landmarks.landmark:
                landmark = results.pose_landmarks.landmark[index]
                right_landmarks.append((int(landmark.x * image_width), int(landmark.y * image_height)))

        # Calculate the direction vector perpendicular to the left arm
        dx = left_landmarks[2][0] - left_landmarks[0][0]
        dy = left_landmarks[2][1] - left_landmarks[0][1]
        magnitude = np.sqrt(dx ** 2 + dy ** 2)
        offset_x = dy / magnitude
        offset_y = -dx / magnitude

        # Draw lines offset from the left arm
        for i in range(len(left_landmarks) - 1):
            start_point = (
                int(left_landmarks[i][0] + offset_x * 30),  # Adjust the offset value (30) for desired distance outside the arm
                int(left_landmarks[i][1] + offset_y * 30)
            )
            end_point = (
                int(left_landmarks[i + 1][0] + offset_x * 30),  # Adjust the offset value (30) for desired distance outside the arm
                int(left_landmarks[i + 1][1] + offset_y * 30)
            )
            cv2.line(image, start_point, end_point, (0, 0, 255), 2)

        # Calculate the direction vector perpendicular to the right arm
        dx = right_landmarks[2][0] - right_landmarks[0][0]
        dy = right_landmarks[2][1] - right_landmarks[0][1]
        magnitude = np.sqrt(dx ** 2 + dy ** 2)
        offset_x = -dy / magnitude
        offset_y = dx / magnitude

        # Draw lines offset from the right arm
        for i in range(len(right_landmarks) - 1):
            start_point = (
                int(right_landmarks[i][0] + offset_x * 30),  # Adjust the offset value (30) for desired distance outside the arm
                int(right_landmarks[i][1] + offset_y * 30)
            )
            end_point = (
                int(right_landmarks[i + 1][0] + offset_x * 30),  # Adjust the offset value (30) for desired distance outside the arm
                int(right_landmarks[i + 1][1] + offset_y * 30)
            )
            cv2.line(image, start_point, end_point, (0, 0, 255), 2)

        # Map 2D left trajectory points to their corresponding depth values
        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        depth_scale = pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
        depth_image = np.asanyarray(depth_frame.get_data())
        left_trajectory_2d = left_landmarks
        left_trajectory_3d = []
        for point in left_trajectory_2d:
            depth_value = depth_image[int(point[1]), int(point[0])] * depth_scale
            point_3d = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [point[0], point[1]], depth_value)
            left_trajectory_3d.append(point_3d)

        # Map 2D right trajectory points to their corresponding depth values
        right_trajectory_2d = right_landmarks
        right_trajectory_3d = []
        for point in right_trajectory_2d:
            depth_value = depth_image[int(point[1]), int(point[0])] * depth_scale
            point_3d = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [point[0], point[1]], depth_value)
            right_trajectory_3d.append(point_3d)

        # Print the 3D trajectories
        print("Left Arm Trajectory (3D):")
        for point in left_trajectory_3d:
            print(f"X: {point[0]}, Y: {point[1]}, Z: {point[2]}")

        print("Right Arm Trajectory (3D):")
        for point in right_trajectory_3d:
            print(f"X: {point[0]}, Y: {point[1]}, Z: {point[2]}")

    # Show the image with landmarks and numbers
    cv2.imshow('Pose Landmarks', image)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the pipeline and close all windows
pipeline.stop()
cv2.destroyAllWindows()
