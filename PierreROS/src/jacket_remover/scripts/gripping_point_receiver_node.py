#!/usr/bin/env python

import rospy
import socket
from geometry_msgs.msg import Point


class GrippingPointReceiverNode:
    def __init__(self):
        # Set up ROS node
        rospy.init_node('gripping_point_receiver_node')

        # Set up socket connection
        self.host = '0.0.0.0'  # IP address of the Windows PC sending gripping points
        self.port = 5353  # Port number used for communication
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((self.host, self.port))
        self.socket.listen(1)

        rospy.loginfo('Waiting for connection...')
        self.client_socket, self.client_address = self.socket.accept()
        rospy.loginfo('Connected to: ' + str(self.client_address))

        # Set up publisher for gripping points
        self.pub = rospy.Publisher('/gripping_point', Point, queue_size=10)

    def receive_gripping_points(self):
        while not rospy.is_shutdown():
            # Receive data from the client
            data = self.client_socket.recv(1024)
            if data:
                # Split the received data into x, y, z values
                gripping_point_values = data.split(',')
                if len(gripping_point_values) == 3:
                    try:
                        # Extract x, y, z values as floats
                        x = float(gripping_point_values[0])
                        y = float(gripping_point_values[1])
                        z = float(gripping_point_values[2])

                        # Create a Point message with the received gripping point
                        gripping_point = Point()
                        gripping_point.x = x
                        gripping_point.y = y
                        gripping_point.z = z

                        # Publish the gripping point
                        self.pub.publish(gripping_point)

                    except ValueError:
                        rospy.logwarn('Received invalid gripping point format.')

    def cleanup(self):
        # Close the socket connection
        self.client_socket.close()
        self.socket.close()


if __name__ == '__main__':
    try:
        # Create an instance of the GrippingPointReceiverNode class
        gripping_point_receiver = GrippingPointReceiverNode()

        # Receive and publish gripping points
        gripping_point_receiver.receive_gripping_points()

        # Cleanup the socket connection
        gripping_point_receiver.cleanup()

    except rospy.ROSInterruptException:
        pass
