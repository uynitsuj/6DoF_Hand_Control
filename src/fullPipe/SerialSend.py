# This is a sample Python script.
import serial
import time
import math as m

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

conv = 180/m.pi

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()
#ser.flush()

def serial_send(j1, j2, j3, j4, j5, j6):
    print('sending')
    streng = "<" + str(j1*conv) + "," + str(j2*conv) + "," + str(j3*conv) + "," + str(j4*conv) + "," + str(j5*conv) + "," + str(j6*conv) + ">\n"
    ser.write(streng.encode('ascii'))
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
