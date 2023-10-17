import rospy
from std_msgs.msg import String
import socket

# ROS node initialization
rospy.init_node('midpoints_publisher', anonymous=True)

# ROS publisher
pub = rospy.Publisher('midpoints_topic', String, queue_size=10)

def publish_midpoints(midpoints):
    # Convert midpoints to a string representation
    midpoints_str = ', '.join(str(coordinate) for coordinate in midpoints)

    # Publish the midpoints as a ROS message
    pub.publish(midpoints_str)
    #rospy.loginfo("Published midpoints: %s", midpoints_str)

# Socket configuration
server_address = '192.168.0.21'
server_port = 5353
buffer_size = 1024

# Create a socket and bind it to the server address and port
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((server_address, server_port))
sock.listen(1)

# Wait for a client to connect
print 'Waiting for a client to connect...'
client_sock, client_address = sock.accept()
print 'Client connected:', client_address

# Main loop
if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            # Receive the message from the client
            message = client_sock.recv(buffer_size)
            print 'Received message:', message

            # Extract the midpoints from the message
            midpoints = [float(coordinate.strip()) for coordinate in message.split(',')]

            # Publish the midpoints
            publish_midpoints(midpoints)

    except rospy.ROSInterruptException:
        pass

    finally:
        # Close the client socket and the server socket
        client_sock.close()
        sock.close()
