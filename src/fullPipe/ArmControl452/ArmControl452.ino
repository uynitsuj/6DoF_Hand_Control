#include <Servo.h>

int angles[] = {180, 180, 180, 180, 180, 180};

Servo j1;
Servo j2;
Servo j3;
Servo j4;
Servo j5;
Servo j6; 

//int getNum(){
//  String num = "";
//  int i = 0;
//  while(Serial.available()){
//    char val = (char)Serial.read();
//    num = num + String(val);
//    i++;
//    if(i == 3){
//      Serial.print(num);
//      break;
//    }
//  }
//  return(num.toInt());
//}

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
String commandString = "";
String i = "";
bool check = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  j1.attach(11);
  j2.attach(10);
  j3.attach(9);
  j4.attach(6);
  j5.attach(5);
  j6.attach(3);
  j1.write(135*(2.0/3.0));
  j2.write(100*(2.0/3.0));
  j3.write(115*(2.0/3.0));
  j4.write(90);
  j5.write(90);
  j6.write(90);

}

void loop() {
  if (stringComplete) {
    stringComplete = false;
    getCommand();
    i = getStep();
//    Serial./println(i);
    if(commandString.equals("jone")){
      int big = int(2.0/3.0 * i.toInt());
      j1.write(big);
      if(check)Serial.println(commandString+i);
    }
    if(commandString.equals("jtwo")){
      int big = int(2.0/3.0 * i.toInt());
      if(big <= 180)j2.write(big);
      if(check)Serial.println(commandString+i);
    }
    if(commandString.equals("jthr")){
      int big = int(2.0/3.0 * i.toInt());
      j3.write(big);
      if(check)Serial.println(commandString+i);
    }
    
    if(commandString.equals("jfou")){
      j4.write(i.toInt());
      if(check)Serial.println(commandString+i);
    }
    if(commandString.equals("jfiv")){
      j5.write(i.toInt());
      if(check)Serial.println(commandString+i);
    }
    if(commandString.equals("jsix")){
      j6.write(i.toInt());
      if(check)Serial.println(commandString+i);
    }
    inputString = "";
  }
  // put your main code here, to run repeatedly:
//    angles[0] = getNum();
//    angles[1] = getNum();
//    angles[2] = getNum();
//    angles[3] = getNum();
//    angles[4] = getNum();
//    angles[5] = getNum();
//    Serial.println();
//    if(Serial.available() > 0 && (char)Serial.read() == '\n'){
//      int big = int(2.0/3.0 * angles[0]);
//      j1.write(big);
//      j2.write(angles[1]);
//      int big2 = int(2.0/3.0 * angles[2]);
//      j3.write(big2);
//      j4.write(angles[3]);
//      j5.write(angles[4]);
//      j6.write(angles[5]);
//    }
//    else{
//      Serial.end();
//      Serial.begin(115200);
//    }
}

String getStep()
{
  String value = inputString.substring(5, inputString.length() - 1);
  return value;
}

void getCommand()
{
  if (inputString.length() > 0)
  {
    commandString = inputString.substring(1, 5);
  }
}

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
