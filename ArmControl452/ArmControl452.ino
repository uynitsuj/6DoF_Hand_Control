#include <Servo.h>

int angles[] = {180, 180, 180, 180, 180};

Servo j1;
Servo j2;
Servo j3;
Servo j4;
Servo j5; 



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  j1.attach(11);
  j2.attach(10);
  j3.attach(9);
  j4.attach(6);
  j5.attach(5);
  
}

void loop() {
  // put your main code here, to run repeatedly:
   if(Serial.available() > 0){
    if(Serial.read() == '<'){
      angles[0] = Serial.parseInt();
      angles[1] = Serial.parseInt();
      angles[2] = Serial.parseInt();
      angles[3] = Serial.parseInt();
      angles[4] = Serial.parseInt();
    }
    if(Serial.read() == '>'){
      Serial.println("correct!");
      for(int i = 0; i < 5; i++){
        Serial.print(angles[i]);
        Serial.print(", ");
      }
    }
    int big = int(2.0/3.0 * angles[0]);
    j1.write(big);
    int big2 = int(2.0/3.0 * angles[1]);
    j2.write(big2);
    j3.write(angles[2]);
    j4.write(angles[3]);
    j5.write(angles[4]);
  }
  
}
