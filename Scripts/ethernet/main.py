# This is a sample Python script.

# Press ⌃R to execute it or replace it with your code.
# Press Double ⇧ to search everywhere for classes, files, tool windows, actions, and settings.


import socket
import numpy as np
import pickle

sok = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ip = "raspberrypi.local"
port = 5005
serverAddress = (ip, port)
sok.connect(serverAddress)
arr = np.empty([2, 2], dtype=float)

while True:
    mes = input('What to send?\n')
    arr[0, 0] = int(mes)
    arr[0, 1] = int(mes) * int(mes)
    arr[1, 0] = int(mes) * 4
    arr[1, 1] = int(mes) / 2
    data_string = pickle.dumps(arr)
    #message = arr.encode()
    sok.send(data_string)
