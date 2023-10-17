#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from baxter_interface import Gripper


class JacketRemoverNode:
    def __init__(self):
        # Set up ROS node
        rospy.init_node('jacket_remover_node')

        # Set up subscribers for gripping points and user command
        rospy.Subscriber('/gripping_point', Point, self.gripping_point_callback)
        rospy.Subscriber('/user_command', String, self.user_command_callback)

        # Set up gripper
        self.gripper = Gripper('left')  # Assuming you're using the left gripper of Baxter

        # Initialize gripping point
        self.gripping_point = None

        # Initialize user command
        self.user_command = None

    def gripping_point_callback(self, msg):
        # Update the gripping point based on the received message
        self.gripping_point = msg

    def user_command_callback(self, msg):
        # Update the user command based on the received message
        self.user_command = msg.data

    def remove_jacket(self):
        # Wait for the gripping point and user command to be available
        while not rospy.is_shutdown() and (self.gripping_point is None or self.user_command is None):
            rospy.sleep(0.1)

        # Check if the user command is valid (e.g., "remove_jacket")
        if self.user_command == "remove_jacket":
            # Open the gripper
            self.gripper.open()

            # Move the robot to the gripping point
            self.move_to_gripping_point(self.gripping_point)

            # Close the gripper to grip the jacket
            self.gripper.close()

            # Move the robot backward to help the person remove the jacket
            self.move_to_gripping_point(self.gripping_point, backward=True)

            # Open the gripper to release the jacket
            self.gripper.open()

    def move_to_gripping_point(self, gripping_point, backward=False):
        # Implement the logic to move the robot's arm to the gripping point
        # You can use MoveIt, inverse kinematics, or any other method that suits your setup

        # Example: Move the arm to the gripping point coordinates
        arm = "left"  # Choose the appropriate arm (left or right) of the Baxter robot
        x = gripping_point.x
        y = gripping_point.y
        z = gripping_point.z

        # Implement the code to move the arm accordingly

        # For example, you can use the BaxterMoveItNode's move_to_pose() method:
        # baxter_moveit.move_to_pose(pose)

        # Adjust the logic based on your specific setup and requirements

        pass

    def cleanup(self):
        # Perform cleanup tasks
        pass


if __name__ == '__main__':
    try:
        # Create an instance of the JacketRemoverNode class
        jacket_remover = JacketRemoverNode()

        # Start the jacket removal process
        jacket_remover.remove_jacket()

        # Perform cleanup tasks
        jacket_remover.cleanup()

    except rospy.ROSInterruptException:
        pass
