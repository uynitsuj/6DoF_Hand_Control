# This is a sample Python script.
import serial

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()

def serial_send(j1, j2, j3, j4, j5,j6):
    ser.write('<'.encode('utf-8'))
    ser.write(j1)
    ser.write(",".encode('utf-8'))
    ser.write(j2)
    ser.write(",".encode('utf-8'))
    ser.write(j3)
    ser.write(",".encode('utf-8'))
    ser.write(j4)
    ser.write(",".encode('utf-8'))
    ser.write(j5)
    ser.write(",".encode('utf-8'))
    ser.write(j6)
    ser.write(",".encode('utf-8'))
    ser.write('>'.encode('utf-8'))
    ser.write("\n".encode('utf-8'))
    # <j1,j2,j3,j4,j5>


print('sending')
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
#serial_send(180,0,180,0,180,0)
#serial_send(0,0,0,0,0,0)
serial_send(180,180,180,180,180,180)