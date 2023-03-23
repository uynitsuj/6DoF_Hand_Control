# This is a sample Python script.
import serial

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
ser = serial.Serial('COM8', 9600, timeout=1)
ser.reset_input_buffer()

def serial_send(j1, j2, j3, j4, j5):
    ser.write("<")
    ser.write(j1.encode('utf-8'))
    ser.write(",")
    ser.write(j2.encode('utf-8'))
    ser.write(",")
    ser.write(j3.encode('utf-8'))
    ser.write(",")
    ser.write(j4.encode('utf-8'))
    ser.write(",")
    ser.write(j5.encode('utf-8'))
    ser.write(">")
    ser.write("\n")
    # <j1,j2,j3,j4,j5>

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
serial_send(12,120,3,46,88)
