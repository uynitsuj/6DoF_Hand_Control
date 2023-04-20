/* 
 *  This is the driver program for the arduino. THe arduino recieves string over serial specifying
 *  a servo to move and the angle to move to. The program also has the ability to send recived string back
 *  for verification.
 */

#include <Servo.h>

// Declare Servo objects for each joint
Servo j1;
Servo j2;
Servo j3;
Servo j4;
Servo j5;
Servo j6;

// Initialize variables for serial communication
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
String commandString = ""; // String specifying which servo to move
String i = ""; // String specifying position to move to
bool check = false; // Sets program to send recieved angles back over serial for debugging

void setup() {
  // Start serial communication at 115200 baud rate
  Serial.begin(115200);

  // Attach the servo objects to the corresponding pins
  j1.attach(11);
  j2.attach(10);
  j3.attach(9);
  j4.attach(6);
  j5.attach(5);
  j6.attach(3);

  // Set initial servo angles
  j1.write(135 * (2.0 / 3.0));
  j2.write(100 * (2.0 / 3.0));
  j3.write(115 * (2.0 / 3.0));
  j4.write(90);
  j5.write(90);
  j6.write(90);
}

void loop() {
  // Check if a complete string has been received
  if (stringComplete) {
    stringComplete = false;
    getCommand();
    i = getStep();

    // Move the servos based on the received command
    if (commandString.equals("jone")) {
      int big = int(2.0 / 3.0 * i.toInt()); // 2/3 factor to allow 270deg servos to access full range of motion (servo class assumes 180deg)
      j1.write(big);
      if (check) Serial.println(commandString + i);
    }
    if (commandString.equals("jtwo")) {
      int big = int(2.0 / 3.0 * i.toInt());
      if (big <= 180) j2.write(big);
      if (check) Serial.println(commandString + i);
    }
    if (commandString.equals("jthr")) {
      int big = int(2.0 / 3.0 * i.toInt());
      j3.write(big);
      if (check) Serial.println(commandString + i);
    }

    if (commandString.equals("jfou")) {
      j4.write(i.toInt());
      if (check) Serial.println(commandString + i);
    }
    if (commandString.equals("jfiv")) {
      j5.write(i.toInt());
      if (check) Serial.println(commandString + i);
    }
    if (commandString.equals("jsix")) {
      j6.write(i.toInt());
      if (check) Serial.println(commandString + i);
    }
    // Clear the inputString for the next command
    inputString = "";
  }
}

// Extract the step value from the received string
String getStep() {
  String value = inputString.substring(5, inputString.length() - 1);
  return value;
}

// Extract the command from the received string
void getCommand() {
  if (inputString.length() > 0) {
    commandString = inputString.substring(1, 5);
  }
}

// Read the incoming serial data
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
