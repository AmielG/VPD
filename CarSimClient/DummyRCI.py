import socket
import numpy as np

while True:
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('127.0.0.1', 60001))
        bytes_received = client_socket.recv(4000)
        array_received = np.frombuffer(bytes_received, dtype=np.float32)  # converting into float array
        print(f"received: {array_received}")
        client_socket.close()
    except Exception as msg:
        print(msg)
