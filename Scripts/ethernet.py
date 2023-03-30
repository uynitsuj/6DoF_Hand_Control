import socket
import pickle
import numpy as np

sok = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ip = "0.0.0.0"
port = 5005
serverAddress = (ip, port)
sok.bind(serverAddress)
sok.listen(1)
print("Waiting for connection")
connection, add = sok.accept()
while True:
    data = connection.recv(2048)
    arr = pickle.loads(data)
    print(arr)