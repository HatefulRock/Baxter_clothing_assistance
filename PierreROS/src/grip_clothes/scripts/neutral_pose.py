import rospy
import moveit_commander

# Initialize the MoveIt Commander
moveit_commander.roscpp_initialize([])

# Create a RobotCommander object to communicate with the robot
robot = moveit_commander.RobotCommander()

# Create a MoveGroupCommander object for the arms
arm_group = moveit_commander.MoveGroupCommander("both_arms")

# Set the named target for the home position
arm_group.set_named_target("both_arms_home")

# Plan and execute the trajectory to move the arms to the home position
arm_group.go(wait=True)

# Cleanup MoveIt Commander
moveit_commander.roscpp_shutdown()
