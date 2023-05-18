import cv2
import socket
import numpy as np

def receive_image_from_socket(sock):
    # Receive the image size first
    img_size_bytes = sock.recv(8)
    img_size = int.from_bytes(img_size_bytes, byteorder='big')

    # Receive the image data
    img_data = b''
    remaining_size = img_size
    while remaining_size > 0:
        data = sock.recv(4096)
        img_data += data
        remaining_size -= len(data)

    # Convert the image data to a NumPy array
    img_array = np.frombuffer(img_data, dtype=np.uint8)

    # Decode the image array
    image = cv2.imdecode(img_array, cv2.IMREAD_UNCHANGED)

    return image

if __name__ == "__main__":
    # Initialize network socket
    host = '0.0.0.0'  # Listen on all available network interfaces
    port = 5353  # Use the same port number as the sender script
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((host, port))
    sock.listen(1)
    #print(f"Listening on {host}:{port}")

    # Accept a connection from the sender
    conn, addr = sock.accept()
    #print(f"Connected to {addr[0]}:{addr[1]}")

    cv2.namedWindow('Receiver - Color', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Receiver - Depth', cv2.WINDOW_NORMAL)
    while True:
        # Receive the color image from the sender
        color_image = receive_image_from_socket(conn)

        # Receive the depth image from the sender
        depth_image = receive_image_from_socket(conn)

        # Display the color and depth images
        cv2.imshow('Receiver - Color', color_image)
        cv2.imshow('Receiver - Depth', depth_image)

        # Press 'q' key to stop
        if cv2.waitKey(1) == ord('q'):
            break

    # Close the connection and socket
    conn.close()
    sock.close()
