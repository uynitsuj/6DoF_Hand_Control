import socket
import pickle
import numpy as np
from IKEngine import IKSixR
from SerialSend import serial_send
import time as time
import quaternion_to_se3 as qse

testing = False
conv = 180/np.pi

if not testing:
    sok = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ip = "0.0.0.0"
    port = 5005
    serverAddress = (ip, port)
    sok.bind(serverAddress)
    sok.listen(1)
    print("Waiting for connection")
    connection, add = sok.accept()
    with connection, connection.makefile('rb') as rfile:
        while True:

            try:
                quataxyz = pickle.load(rfile)
                quat = (quataxyz[0][0], quataxyz[0][1], quataxyz[0][2], quataxyz[0][3])
                quat = qse.normalize_quaternion(quat)
                pose = qse.quaternion_to_se3(quat)
                pose[0][3] = quataxyz[0][4]
                pose[1][3] = quataxyz[0][5]
                pose[2][3] = quataxyz[0][6]
                
#                 pose = np.array(pose)*10
            except:  # Throws exception if incomplete or socket closed
                print("Fuck")
                break
    #         print(pose)
        #     data = connection.recv(4096)
        #     pose = pickle.loads(data)
        #     pose = np.array(pose)
#             print(pose)

            robot = IKSixR(0, H06=(
                pose[0][0], pose[0][1], pose[0][2], pose[0][3], pose[1][0], pose[1][1], pose[1][2], pose[1][3], pose[2][0], pose[2][1], pose[2][2], pose[2][3]))
            robot.IK()
            try:
        #         # robot.draw_limbs(tbn=1)
        #         # robot.draw_limbs(tbn=2)
        #         # robot.draw_limbs(tbn=3)
        #         # robot.draw_limbs(tbn=4)
        #         # robot.draw_limbs(tbn=5)
        #         # robot.draw_limbs(tbn=6)
        #         # robot.draw_limbs(tbn=7)
        # #         robot.draw_limbs(tbn=8)
#                 print(str(robot.rtnposeang(1, 1)*conv) + " "+str(robot.rtnposeang(2, 6)*conv)+" "+str(robot.rtnposeang(
#                  3, 6)*conv)+" "+str(robot.rtnposeang(4, 6)*conv)+" "+str(robot.rtnposeang(5, 3)*conv)+" "+str(robot.rtnposeang(6, 3)*conv))
                serial_send(robot.rtnposeang(1, 1),robot.rtnposeang(2, 6),robot.rtnposeang(
                    3, 6),robot.rtnposeang(4, 6),robot.rtnposeang(5, 3),robot.rtnposeang(6, 3))
        #         
        # 
            except IndexError:
                print(" Out of config space")

            # print("theta1"+robot.rtnposeang(1, 7) + " "+robot.rtnposeang(2, 7)+" "+robot.rtnposeang(
            #    3, 7)+" "+robot.rtnposeang(4, 7)+" "+robot.rtnposeang(5, 7)+" "+robot.rtnposeang(6, 7))
else:
    deg = 0
    while True:
        serial_send(deg*(np.pi/180),100*(np.pi/180),30*(np.pi/180),8,70,45)
        deg += 10
        if deg > 270:
            deg = 30
        time.sleep(3)
        
