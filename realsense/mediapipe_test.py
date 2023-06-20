import cv2
import mediapipe as mp
import pyrealsense2 as rs
import numpy as np

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

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
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

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
        
        
    # Define the landmark indices for the left and right sides
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

    # Calculate the direction vector perpendicular to the arm
    dx = left_landmarks[2][0] - left_landmarks[0][0]
    dy = left_landmarks[2][1] - left_landmarks[0][1]
    magnitude = np.sqrt(dx ** 2 + dy ** 2)
    offset_x = dy / magnitude
    offset_y = -dx / magnitude

    # Draw lines offset from the left arm
    for i in range(len(left_landmarks) - 1):
        start_point = (
            int(left_landmarks[i][0] + offset_x * 30),  # Adjust the offset value (10) for desired distance outside the arm
            int(left_landmarks[i][1] + offset_y * 30)
        )
        end_point = (
            int(left_landmarks[i + 1][0] + offset_x * 30),  # Adjust the offset value (10) for desired distance outside the arm
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
            int(right_landmarks[i][0] + offset_x * 30),  # Adjust the offset value (10) for desired distance outside the arm
            int(right_landmarks[i][1] + offset_y * 30)
        )
        end_point = (
            int(right_landmarks[i + 1][0] + offset_x * 30),  # Adjust the offset value (10) for desired distance outside the arm
            int(right_landmarks[i + 1][1] + offset_y * 30)
        )
        cv2.line(image, start_point, end_point, (0, 0, 255), 2)




    # Show the image with landmarks and numbers
    cv2.imshow('Pose Landmarks', image)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the pipeline and close all windows
pipeline.stop()
cv2.destroyAllWindows()
