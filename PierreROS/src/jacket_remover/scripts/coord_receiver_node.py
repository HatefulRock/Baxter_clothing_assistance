#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import zmq

def zmq_subscriber():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://192.168.0.172:10001")
    socket.setsockopt(zmq.SUBSCRIBE, '')

    rospy.init_node('body_frame_subscriber', anonymous=True)
    pub = rospy.Publisher('body_frame_data', String, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            body_frame_data = socket.recv()
            pub.publish(body_frame_data)
            rate.sleep()
        except zmq.error.ContextTerminated:
            break

if __name__ == '__main__':
    try:
        zmq_subscriber()
    except rospy.ROSInterruptException:
        pass
