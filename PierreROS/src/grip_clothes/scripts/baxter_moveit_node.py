#!/usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


class BaxterMoveItNode:
    def __init__(self):
        # Initialize the MoveIt Commander
        moveit_commander.roscpp_initialize(sys.argv)

        # Create a RobotCommander object to communicate with the robot
        self.robot = moveit_commander.RobotCommander()

        # Create a PlanningSceneInterface object to add/remove objects from the scene
        self.scene = moveit_commander.PlanningSceneInterface()

        # Create a MoveGroupCommander object to control the robot's arm
        self.group = moveit_commander.MoveGroupCommander("both_arms")

        # Set the planning time
        self.group.set_planning_time(10)

        # Set the maximum number of planning attempts
        self.group.set_num_planning_attempts(5)

    def move_to_home(self):
        # Move both arms to the home position
        self.group.set_named_target("both_arms_home")
        self.group.go(wait=True)

    def move_to_pose(self, pose):
        # Move the robot's arms to a specified pose
        self.group.set_pose_target(pose)
        self.group.go(wait=True)

    def cleanup(self):
        # Shutdown MoveIt Commander
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('baxter_moveit_node')

        # Create an instance of the BaxterMoveItNode class
        baxter_moveit = BaxterMoveItNode()

        # Move to the home position
        baxter_moveit.move_to_home()

        # Example: Move to a specified pose
        pose = geometry_msgs.msg.Pose()
        # Set the pose values accordingly
        pose.position.x = 0.5
        pose.position.y = 0.2
        pose.position.z = 0.7
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        # Move to the specified pose
        baxter_moveit.move_to_pose(pose)

        # Cleanup MoveIt Commander
        baxter_moveit.cleanup()

    except rospy.ROSInterruptException:
        pass
