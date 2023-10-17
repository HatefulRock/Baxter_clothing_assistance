#!/usr/bin/env python
# -*- coding: utf-8 -*-

# baxter_controller.py: baxter controller
# Author: Ravi Joshi
# Date: 2017/11/17

# import modules
import rospy
import numpy as np
import baxter_interface
from std_msgs.msg import Float64
from threading import currentThread
from baxter_pykdl import baxter_kinematics
from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import JointCommand
from learn_kinematics.srv import fast_kinematics_service


class BaxterController():
    def __init__(self, need_ik=False, joint_speed=0.3):
        self.almost_close = 0.02  # 2cm
        freq = 100  # Hz
        self.rate = rospy.Rate(freq)
        self.is_running = False

        # ratio of maximum joint speed for execution default= 0.3, range= [0.0-1.0]
        self.joint_speed = Float64(joint_speed)

        if need_ik:
            self.ik_service_name = 'fast_kinematics_service'
            rospy.logdebug('Waiting for service %s' % self.ik_service_name)
            rospy.wait_for_service(self.ik_service_name)
            rospy.logdebug('Found service %s' % self.ik_service_name)

        self.left_arm = baxter_interface.Limb('left')
        self.right_arm = baxter_interface.Limb('right')

        self.left_arm_kin = baxter_kinematics('left')
        self.right_arm_kin = baxter_kinematics('right')

        self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self.right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)

        self.left_command = JointCommand()
        self.left_command.names = self.left_arm.joint_names()
        self.left_command.mode = JointCommand.POSITION_MODE

        self.right_command = JointCommand()
        self.right_command.names = self.right_arm.joint_names()
        self.right_command.mode = JointCommand.POSITION_MODE

        left_joint_command_topic = '/robot/limb/left/joint_command'
        right_joint_command_topic = '/robot/limb/right/joint_command'

        self.left_pub = rospy.Publisher(
            left_joint_command_topic, JointCommand, tcp_nodelay=True, queue_size=1)
        self.right_pub = rospy.Publisher(
            right_joint_command_topic, JointCommand, tcp_nodelay=True, queue_size=1)

        left_joint_speed_topic = '/robot/limb/left/set_speed_ratio'
        right_joint_speed_topic = '/robot/limb/right/set_speed_ratio'

        self.left_speed_pub = rospy.Publisher(
            left_joint_speed_topic, Float64, queue_size=1)
        self.right_speed_pub = rospy.Publisher(
            right_joint_speed_topic, Float64, queue_size=1)

    def get_current_joint_angles(self):
        left_joint_angles_dict = self.left_arm.joint_angles()
        left_current_joint_angles = [
            left_joint_angles_dict[joint_name] for joint_name in self.left_command.names]

        right_joint_angles_dict = self.right_arm.joint_angles()
        right_current_joint_angles = [
            right_joint_angles_dict[joint_name] for joint_name in self.right_command.names]

        return left_current_joint_angles + right_current_joint_angles

    def ik(self, pose_both, joint_seed_both=None):
        if joint_seed_both is None:
            joint_seed_both = self.get_current_joint_angles()

        # source: https://github.com/ShibataLab/learn_kinematics/blob/master/scripts/ik_test.py#L8
        new_pose_both = list(pose_both)
        new_pose_both[2] += 0.01
        new_pose_both[9] += 0.01

        try:
            service_request_both = rospy.ServiceProxy(
                self.ik_service_name, fast_kinematics_service, persistent=True)

            '''
            pose is an array consisting of 14 elements.
            First 7 elements represents the end-effector pose of left arm
            Later 7 elements represents the end-effector pose of right arm
            Note that the pose contains position cartesian system and orientaion in quaternion system
            Example:
            pose = [poselinear_left_x,poselinear_left_y,poselinear_left_z,poseangle_left_x,poseangle_left_y,poseangle_left_z,poseangle_left_w,
              poselinear_right_x,poselinear_right_y,poselinear_right_z,poseangle_right_x,poseangle_right_y,poseangle_right_z,poseangle_right_w]

            joint_seed is an array consisting of 14 elements.
            First 7 elements represents the joint angle of left arm
            Later 7 elements represents the joint angle of right arm
            Example:
            joint_seed = [left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2,
                right_s0,right_s1,right_e0,right_e1,right_w0,right_w1,right_w2]
            '''
            service_response_both = service_request_both(
                new_pose_both,  joint_seed_both)
            # sucess code from ik solver is 1
            success = service_response_both.success_left == service_response_both.success_right == 1
            return success, service_response_both.joint_angles
        except rospy.ServiceException, e:
            rospy.logerr('IK service call failed: %s' % e)
            return None

    def move_in_ee_frame(self, both_target_pose):
        success, both_joint_angles = self.ik(both_target_pose)
        if not success:
            return

        self.left_command.command = both_joint_angles[:7]
        self.right_command.command = both_joint_angles[7:]

        left_target_position = both_target_pose[: 3]
        right_target_position = both_target_pose[7:10]

        while not rospy.is_shutdown():
            left_current_position = np.array(
                self.left_arm.endpoint_pose()['position'])
            right_current_position = np.array(
                self.right_arm.endpoint_pose()['position'])

            left_distance = np.linalg.norm(
                left_current_position - left_target_position)
            right_distance = np.linalg.norm(
                right_current_position - right_target_position)

            if (left_distance < self.almost_close) and (right_distance < self.almost_close):
                break

            self.publish_command()
            self.rate.sleep()

    def move_in_joint_frame(self, both_joint_angles):
        self.left_command.command = both_joint_angles[:7]
        self.right_command.command = both_joint_angles[7:]

        left_joint_state = dict(
            zip(self.left_command.names,  self.left_command.command))
        right_joint_state = dict(
            zip(self.right_command.names, self.right_command.command))

        left_target_position = self.left_arm_kin.forward_position_kinematics(
            left_joint_state)[:3]
        right_target_position = self.right_arm_kin.forward_position_kinematics(
            right_joint_state)[:3]

        while not rospy.is_shutdown():
            left_current_position = np.array(
                self.left_arm.endpoint_pose()['position'])
            right_current_position = np.array(
                self.right_arm.endpoint_pose()['position'])

            left_distance = np.linalg.norm(
                left_current_position - left_target_position)
            right_distance = np.linalg.norm(
                right_current_position - right_target_position)

            if (left_distance < self.almost_close) and (right_distance < self.almost_close):
                break

            self.publish_command()
            self.rate.sleep()

    def publish_current_joint_values(self):
        current_joint_angles = self.get_current_joint_angles()
        self.left_command.command = current_joint_angles[:7]
        self.right_command.command = current_joint_angles[7:]
        self.publish_command()

    def move_async_in_ee_frame(self, both_target_pose):
        success, both_joint_angles = self.ik(both_target_pose)
        if not success:
            return

        self.left_command.command = both_joint_angles[:7]
        self.right_command.command = both_joint_angles[7:]

        left_target_position = both_target_pose[: 3]
        right_target_position = both_target_pose[7:10]

        while not rospy.is_shutdown() and getattr(currentThread(), 'do_run', True):
            left_current_position = np.array(
                self.left_arm.endpoint_pose()['position'])
            right_current_position = np.array(
                self.right_arm.endpoint_pose()['position'])

            left_distance = np.linalg.norm(
                left_current_position - left_target_position)
            right_distance = np.linalg.norm(
                right_current_position - right_target_position)

            if (left_distance < self.almost_close) and (right_distance < self.almost_close):
                break

            self.publish_command()
            self.rate.sleep()

    def move_async_in_joint_frame(self, both_joint_angles):
        self.is_running = True
        self.left_command.command = both_joint_angles[:7]
        self.right_command.command = both_joint_angles[7:]

        left_joint_state = dict(
            zip(self.left_command.names,  self.left_command.command))
        right_joint_state = dict(
            zip(self.right_command.names, self.right_command.command))

        left_target_position = self.left_arm_kin.forward_position_kinematics(
            left_joint_state)[:3]
        right_target_position = self.right_arm_kin.forward_position_kinematics(
            right_joint_state)[:3]

        while not rospy.is_shutdown() and getattr(currentThread(), 'do_run', True):
            left_current_position = np.array(
                self.left_arm.endpoint_pose()['position'])
            right_current_position = np.array(
                self.right_arm.endpoint_pose()['position'])

            left_distance = np.linalg.norm(
                left_current_position - left_target_position)
            right_distance = np.linalg.norm(
                right_current_position - right_target_position)

            if (left_distance < self.almost_close) and (right_distance < self.almost_close):
                break

            self.publish_command()
            self.rate.sleep()
        self.is_running = False

    def publish_command(self):
        try:
            self.left_speed_pub.publish(self.joint_speed)
            self.right_speed_pub.publish(self.joint_speed)
            self.left_pub.publish(self.left_command)
            self.right_pub.publish(self.right_command)
        except rospy.exceptions.ROSException as e:
            rospy.logdebug(e.message)

    def control_grippers(self, open=True):
        if open:
            self.left_gripper.open()
            self.right_gripper.open()
        else:
            self.left_gripper.close()
            self.right_gripper.close()

    def __del__(self):
        self.left_pub.unregister()
        self.right_pub.unregister()
        self.left_speed_pub.unregister()
        self.right_speed_pub.unregister()
