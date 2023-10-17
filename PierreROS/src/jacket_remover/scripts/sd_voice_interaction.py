#!/usr/bin/env python

import rospy
from numpy import loadtxt
from baxter_controller import BaxterController
import baxter_interface
from threading import Thread
import threading
import sys

class Phase1:
    def __init__(self):
        self.baxter = BaxterController(need_ik=True, joint_speed=0.15)
        self.thread = None
        self.indices = range(1, 15)

    def move_position1(self, joint_trajectory):
        reshape_trajectory = loadtxt(joint_trajectory, delimiter=',', skiprows=1, usecols=self.indices)
        def move_in_joint_frame_thread(joint_trajectory):
            rospy.sleep(15)
            for joint_angle in joint_trajectory:
                self.baxter.move_async_in_joint_frame(joint_angle)
            rospy.sleep(0.5)
        self.start_thread(move_in_joint_frame_thread, reshape_trajectory)

    def start_thread(self, function_to_call, args_to_function):
        if self.thread is None or not self.thread.isAlive():
            self.thread = Thread(target=function_to_call,
                                 args=(args_to_function,))
            self.thread.start()

if __name__ == '__main__':
    rospy.init_node("voice_interaction")
    print ('Program Start!!!!!!!!')
    phase1 = Phase1()
    phase1.move_position1('./phase2')
