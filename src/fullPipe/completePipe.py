import socket
import pickle
import numpy as np
from multiprocessing import Process, SimpleQueue, shared_memory
import Calibrate.cameracalibrate as cal
import sys
from IKEngine import IKSixR
from SerialSend import serial_send

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
    arrList = pickle.loads(data)
    pose = np.array(arrList)

    robot = IKSixR(self.w, H06=(
        pose[0][0], pose[0][1], pose[0][2], pose[0][3], pose[1][0], pose[1][1], pose[1][2], pose[1][3], pose[2][0], pose[2][1], pose[2][2], pose[2][3]))
    robot.IK()
    try:
        # robot.draw_limbs(tbn=1)
        # robot.draw_limbs(tbn=2)
        # robot.draw_limbs(tbn=3)
        # robot.draw_limbs(tbn=4)
        # robot.draw_limbs(tbn=5)
        # robot.draw_limbs(tbn=6)
        # robot.draw_limbs(tbn=7)
        robot.draw_limbs(tbn=8)
        print(str(robot.rtnposeang(1, 1)) + " "+str(robot.rtnposeang(2, 7))+" "+str(robot.rtnposeang(
            3, 7))+" "+str(robot.rtnposeang(4, 7))+" "+str(robot.rtnposeang(5, 3))+" "+str(robot.rtnposeang(6, 3)))
        serial_send(robot.rtnposeang(1, 1),robot.rtnposeang(2, 7),robot.rtnposeang(
            3, 7),robot.rtnposeang(4, 7),robot.rtnposeang(5, 3),robot.rtnposeang(6, 3))
        

    except IndexError:
        print(" Out of config space")

    # print("theta1"+robot.rtnposeang(1, 7) + " "+robot.rtnposeang(2, 7)+" "+robot.rtnposeang(
    #    3, 7)+" "+robot.rtnposeang(4, 7)+" "+robot.rtnposeang(5, 7)+" "+robot.rtnposeang(6, 7))
