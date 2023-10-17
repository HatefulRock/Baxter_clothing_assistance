import socket
import numpy as np

# Define the server address and port
server_address = '192.168.0.21'
server_port = 5353

# Create a socket object
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the server address and port
sock.bind((server_address, server_port))

# Listen for incoming connections
sock.listen(1)

print("Waiting for a connection...")

# Accept a connection from a client
client_socket, client_address = sock.accept()

print("Connected to:", client_address)

while True:
    # Receive data from the client
    data = client_socket.recv(1024)

    if not data:
        # Connection closed by the client
        break

    # Print the raw received message
    print("Raw received message:", data.strip())
    # Parse the received message into a list of floats
    midpoints = []
    coordinates = data.strip()[1:-1].split(',')
    for coordinate in coordinates:
        midpoints.append(float(coordinate.strip()))

    # Print the received coordinates
    print("Received coordinates:")
    for i in range(0, len(midpoints), 3):
        x = midpoints[i]
        y = midpoints[i+1]
        z = midpoints[i+2]
        print("X: %.3f, Y: %.3f, Z: %.3f" % (x, y, z))



# Close the client socket
client_socket.close()

# Close the server socket
sock.close()
