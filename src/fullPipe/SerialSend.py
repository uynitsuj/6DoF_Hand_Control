# Import necessary libraries
import serial
import math as m

# Conversion factor from radians to degrees
conv = 180/m.pi

# Configure serial communication
serialcomm = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
serialcomm.bytesize = serial.EIGHTBITS
serialcomm.parity = serial.PARITY_NONE
serialcomm.stopbits = serial.STOPBITS_ONE
serialcomm.timeout = 2

# Function to send joint angles to the robotic arm via serial communication


def serial_send(j1, j2, j3, j4, j5, j6):
    print('sending')

    # Convert joint angles from radians to degrees and format as strings
    jone = '#jone' + str(int(j1*conv))
    jtwo = '#jtwo' + str(int(j2 * conv)+180)
    jthree = '#jthr' + str(int(j3 * conv)*-1+115)
    jfour = '#jfou' + str(int(j4 * conv)+180)
    jfive = '#jfiv' + str(int(j5 * conv)+180)
    jsix = '#jsix' + str(int(j6 * conv)+180)

    # Send joint angles to the robotic arm via serial communication
    serialcomm.write((jone + '\n').encode('ascii'))
    serialcomm.write((jtwo + '\n').encode('ascii'))
    serialcomm.write((jthree + '\n').encode('ascii'))
    serialcomm.write((jfour + '\n').encode('ascii'))
    serialcomm.write((jfive + '\n').encode('ascii'))
    serialcomm.write((jsix + '\n').encode('ascii'))

    print('sent')
