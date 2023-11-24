#include <ESP32Servo.h>

Servo Servo1; 
Servo Servo2; // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos1 = 0;
int pos2 = 0; 
; // variable to store the servo position
int down_angle = 0;
int up_angle = 100;
int tx = 10;
int ts = 0;
int Pair1 = 0;
int Pair2 = 0;
bool active = 0;

void setup() {
  Serial.begin(9600);
  Servo1.attach(5);
  Servo2.attach(6);  // attaches the servo on pin 9 to the servo object

}

void loop() {
  ts = tx*1000/90;
  if (Serial.available() >= 3) {
     Pair1 = Serial.parseInt(); //integer value from the first part of the serial msg
     Pair2 = Serial.parseInt(); //integer value from the second part of the serial msg
     Serial.println(Pair1);
     active = 1;

  }
  if(Pair1 == 1 && active == 1){
     active = 0;
  for (pos1 = down_angle; pos1 <= up_angle; pos1 += 1) { 
    Servo1.write(pos1);              // tell servo to go to position in variable 'pos'
    delay(ts);                       
  }}
  if(Pair1 == 2 && active == 1){
    active =0;
  for (pos1 = up_angle; pos1 >= down_angle; pos1 -= 1) { 
    Servo1.write(pos1);              // tell servo to go to position in variable 'pos'
    delay(ts);    
  }}

}

