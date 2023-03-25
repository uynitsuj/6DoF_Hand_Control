# This is a sample Python script.
import serial
import time
import math as m

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

conv = 180/m.pi

# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
# ser.reset_input_buffer()
serialcomm = serial.Serial('/dev/ttyACM0', 115200, timeout = 0)
serialcomm.bytesize = serial.EIGHTBITS
serialcomm.parity = serial.PARITY_NONE
serialcomm.stopbits = serial.STOPBITS_ONE
serialcomm.timeout = 2
# time.sleep(2)
#ser.flush()

def serial_send(j1, j2, j3, j4, j5, j6):
    print('sending')

    jone = '#jone' + str(int(j1*conv)+180)
    serialcomm.write((jone + '\n').encode('ascii'))

    jtwo = '#jtwo' + str(int(j2 * conv)+180)
    serialcomm.write((jtwo + '\n').encode('ascii'))

    jthree = '#jthr' + str(int(j3 * conv)*-1+115)
    serialcomm.write((jthree + '\n').encode('ascii'))

    jfour = '#jfou' + str(int(j4 * conv)+180)
    serialcomm.write((jfour + '\n').encode('ascii'))

    jfive = '#jfiv' + str(int(j5 * conv)+180)
    serialcomm.write((jfive + '\n').encode('ascii'))

    jsix = '#jsix' + str(int(j6 * conv)+180)
    serialcomm.write((jsix + '\n').encode('ascii'))

#     arr = [j1, j2, j3, j4, j5, j6]
#     streng = ""
#     for i in range(6):
#         if i == 1:
#             j = str(int(arr[i]*conv) + 360)
#         else:
#             j = str(int(arr[i]*conv) + 180)
#         if 3 - len(j):
#             for i in range(3 - len(j)):
#                 j = "0" + j
#         streng = streng + j
#     streng = streng + "\n"
#
# #     streng = "<" + str(int(j1*conv)) + "," + str(int(j2*conv)) + "," + str(int(j3*conv)) + "," + str(int(j4*conv)) + "," + str(int(j5*conv)) + "," + str(int(j6*conv)) + ">\n"
#     print(streng)
#     start = time.time()
#     ser.write(streng.encode('ascii'))
#     end = time.time()
#     delta = end - start
#     print(delta)
    print('sent')
    
    #ser.close()
#     ser.write('<'.encode('utf-8'))
#     ser.write(j1)
#     ser.write(",".encode('utf-8'))
#     ser.write(j2)
#     ser.write(",".encode('utf-8'))
#     ser.write(j3)
#     ser.write(",".encode('utf-8'))
#     ser.write(j4)
#     ser.write(",".encode('utf-8'))
#     ser.write(j5)
#     ser.write(",".encode('utf-8'))
#     ser.write('>'.encode('utf-8'))
#     ser.write("\n".encode('utf-8'))
    #ser.open()
    # <j1,j2,j3,j4,j5>



# See PyCharm help at https://www.jetbrains.com/help/pycharm/
#serial_send(180,0,180,0,180,0)
#serial_send(0,0,0,0,0,0)

# while True:
#     serial_send(180,180,180,180,180)
#     read_serial=ser.readline().decode('ascii').rstrip()
#     print(read_serial)
#     read_serial=ser.readline().decode('ascii').rstrip()
#     print(read_serial)
#     time.sleep(5)
#     serial_send(0,180,180,180,180)
#     read_serial=ser.readline().decode('ascii').rstrip()
#     print(read_serial)
#     read_serial=ser.readline().decode('ascii').rstrip()
#     print(read_serial)
#     time.sleep(5)
