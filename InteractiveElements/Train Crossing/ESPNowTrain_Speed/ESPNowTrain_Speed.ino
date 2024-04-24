#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>



typedef struct DataStruct {
  int Pair1;
  int Pair2;
  int Pair3;
  int Pair4;
  int Pair5;
  int Pair6;
  int Pair7;
  int Pair8;
  int Pair9;
  int Pair10;
  int Pair11;
  int Pair12;
  int Pair13;
  int Pair14;
  int Pair15;
  int Pair16;
}DataStruct;
DataStruct Data;

/* PCB Connections for Servo Adapter
Connector 1      Connector 2     Connector 3      Connector 4
LED side1 12              15              6                21 
Led Side2 14              4            Linked 1           Linked 2

Servo 1  25              19              8                23
Servo 2 27                5              7                22
*/
/* PCB Connections/ V2
Connector 1      Connector 2     Connector 3      Connector 4
Led Side 1 12         32              16                21 
Led Side 2 14         4            Linked 1           Linked 2

Servo 1 33            19              17                23
Servo 2 27            5              16                22
*/

/* Servo Pinouts per connector.
        J1    J2   LED1    LED2
Conn1   33    27   
Conn2   19    5
Conn3   17    16
Conn4   23    22
*/
Servo servo1, servo2, servo3, servo4;
#define CHANNEL 1
#define led1 12
#define led2 14
#define led3 32
#define led4 4
#define servopin1 33
#define servopin2 19
#define servopin3 17
#define servopin4 23
#define maxangle 150
#define minangle 0
#define timestep 40
#define swapspeed 6

int angle1 = 0;
int angle2 = 0;
int angle3 = 0;
int angle4 = 0;

int cal1 = 0;
int cal2 = 0;
int cal3 = 0;
int cal4 = 0;

int shift = 180;


void setup() {
  Serial.begin(9600);
  servo1.attach(servopin1);
  servo2.attach(servopin2);
  servo3.attach(servopin3);
  servo4.attach(servopin4);
  
  delay(100);
  servo1.write(angle1);
  servo2.write(180-angle2);
  servo3.write(angle3);
  servo4.write(180-angle4);
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;}
  esp_now_register_recv_cb(OnDataRecv);
 
}



void loop() {
  //angle1 = servoangle(Data.Pair9,servo1,servo2,angle1,led1,led2);
  //angle2 = servoangle(Data.Pair11,servo3,servo4,angle2,led3,led4);

  //#angle3 = servoangle(Data.Pair11,servo3,servo4,angle3,led3,led4);
  //angle4 = servoangle(Data.Pair12,servo3,servo4,angle4,led3,led4);

  angle1 = Data.Pair9+cal1;
  angle2 = shift-Data.Pair10+cal2;
  angle3 = Data.Pair11+cal3;
  angle4 = shift-Data.Pair12+cal4;

  servo1.write(angle1);
  servo2.write(angle2);
  delay(100);
  servo3.write(angle3);
  servo4.write(angle4);
  delay(100);
  //analogWrite(led1, 0);
  //analogWrite(led2, 0);
  //analogWrite(led3, 0);
  //analogWrite(led4, 0);
}


int servoangle(int dest, Servo &obj, Servo &obj2, int angle, int l1, int l2) {
  if (dest >= maxangle) {
    dest = maxangle;
  }
  if (dest <= minangle) {
    dest = minangle;
  }
  if (angle < dest) {

    for (int i = angle; i <= dest; i++) {
      angle = i;
      obj.write(angle);
      obj2.write(180-angle);
      delay(timestep);
      trainled(i, l1, l2);
    }
  }

  if (angle > dest) {

    for (int i = angle; i >= dest; i--) {
      angle = i;
      obj.write(angle);
      obj2.write(180-angle);
      delay(timestep);
      trainled(i, l1, l2);
    }
  }

  return angle;
}

void trainled(int in, int l1, int l2) {
  if (in / 6 % 2 == 0) {
    analogWrite(l1, 255);
    analogWrite(l2, 0);
  } else {
    analogWrite(l1, 0);
    analogWrite(l2, 255);
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Data, incomingData, sizeof(Data));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Relevant Data: ");
  Serial.print(Data.Pair9);
  Serial.print(", ");
  Serial.print(Data.Pair10);
  Serial.print(", ");
  Serial.print(Data.Pair11);
  Serial.print(", ");
  Serial.println(Data.Pair12);
}