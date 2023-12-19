#include <ESP32Servo.h>

Servo servo1, servo2;


#define led1 18
#define led2 19
#define led3 32
#define led4 33
#define servopin1 5
#define servopin2 16 
#define maxangle 150
#define minangle 0
#define timestep 40
#define swapspeed 6

int angle1 = 0;
int angle2 = 0;
int Pair1 = 0;
int Pair2 = 0;
int Pair3 = 0;
int Pair4 = 0;


void setup() {
  Serial.begin(9600);
  servo1.attach(servopin1);
  servo2.attach(servopin2);
  delay(20);
  servo1.write(angle1);
  servo2.write(angle2);

  // put your setup code here, to run once:

}

void loop() {
  if (Serial.available() >= 3) {
    Pair1 = Serial.parseInt();
    Pair2 = Serial.parseInt();
    Pair3 = Serial.parseInt();
    Pair4 = Serial.parseInt();

    angle1 = servoangle(Pair1,servo1,servo2,angle1,led1,led2);
    //Serial.print("Servo pair 1 to Angle: ");
    //Serial.println(Pair1);
    //angle2 = servoangle(Pair2,servo2,angle2,led3,led4);
    //Serial.print("Servo pair 2 to Angle: ");
    //Serial.println(Pair2);
    }
  
delay(100);
analogWrite(led1,0);
analogWrite(led2,0);
analogWrite(led3,0);
analogWrite(led4,0);
}

int servoangle(int dest, Servo &obj, Servo &obj2, int angle, int l1, int l2){
  if(dest >= maxangle){dest = maxangle;}
  if(dest <= minangle){dest = minangle;}
  if(angle < dest){
      
    for(int i = angle; i <= dest; i++){
      angle = i;
      obj.write(angle);
      obj2.write(angle);
      delay(timestep);
      trainled(i,l1,l2);
      }}

  if(angle > dest){
    
    for(int i = angle; i >= dest; i--){
      angle = i;
      obj.write(angle);
      obj2.write(angle);
      delay(timestep);
      trainled(i,l1,l2);}}

  return angle;
  }


void trainled(int in, int l1, int l2){
        if(in/6 % 2 == 0){
        analogWrite(l1,255);
        analogWrite(l2,0);
      }
      else{
        analogWrite(l1,0);
        analogWrite(l2,255);
      }
}
