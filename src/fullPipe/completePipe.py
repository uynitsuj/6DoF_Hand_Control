# This is the driver file for the raspberry pi. It recieves data over ethernet
# corresponding to a pose that is used for Inverse Kinematics. The IK solution is
# then sent to the arduino via serial.

import socket
import pickle
import numpy as np
from IKEngine import IKSixR
from SerialSend import serial_send
import time as time
import quatlinalg as qse

testing = False
conv = 180/np.pi

# Establish a TCP/IP socket connection to receive quaternion data
if not testing:
    sok = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ip = "0.0.0.0"
    port = 5005
    serverAddress = (ip, port)
    sok.bind(serverAddress)
    sok.listen(1)
    print("Waiting for connection")
    connection, add = sok.accept()

    # Receive and process quaternion data until the socket is closed
    with connection, connection.makefile('rb') as rfile:
        while True:
            try:
                # Load quaternion data from the socket and convert to SE(3) pose
                quataxyz = pickle.load(rfile)
                quat = (quataxyz[0][0], quataxyz[0][1],
                        quataxyz[0][2], quataxyz[0][3])
                quat = qse.normalize_quaternion(quat)
                pose = qse.quaternion_to_se3(quat)
                pose[0][3] = quataxyz[0][4]
                pose[1][3] = quataxyz[0][5]
                pose[2][3] = quataxyz[0][6]
            except:
                # Throws exception if incomplete or socket closed
                print("Pipe End")
                break

            try:
                # Perform inverse kinematics to compute joint angles and send to robot controller
                robot = IKSixR(0, H06=(
                    pose[0][0], pose[0][1], pose[0][2], pose[0][3], pose[1][0], pose[1][1], pose[1][2], pose[1][3], pose[2][0], pose[2][1], pose[2][2], pose[2][3]))
                robot.IK()
                serial_send(robot.rtnposeang(1, 1), robot.rtnposeang(2, 6), robot.rtnposeang(
                    3, 6), robot.rtnposeang(4, 6), robot.rtnposeang(5, 3), robot.rtnposeang(6, 3))
            except IndexError:
                # Catch exception if inverse kinematics fails due to being out of configuration space
                print(" Out of config space")

# Testing mode for generating dummy data
else:
    deg = 0
    while True:
        serial_send(deg*(np.pi/180), 100*(np.pi/180),
                    30*(np.pi/180), 8, 70, 45)
        deg += 10
        if deg > 270:
            deg = 30
        time.sleep(3)
