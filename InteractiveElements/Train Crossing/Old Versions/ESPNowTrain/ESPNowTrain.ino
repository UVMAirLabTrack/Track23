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
}DataStruct;
DataStruct Data;

Servo servo1, servo2;
#define CHANNEL 1
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


void setup() {
  Serial.begin(9600);
  servo1.attach(servopin1);
  servo2.attach(servopin2);
  delay(100);
  servo1.write(angle1);
  servo2.write(angle2);
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;}
  esp_now_register_recv_cb(OnDataRecv);
 
}



void loop() {
  angle1 = servoangle(Data.Pair5,servo1,servo2,angle1,led1,led2);
  //angle2 = servoangle(Data.Pair6,servo3,servo4,angle2,led3,led4);
  delay(100);
  analogWrite(led1, 0);
  analogWrite(led2, 0);
  analogWrite(led3, 0);
  analogWrite(led4, 0);
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
      obj2.write(angle);
      delay(timestep);
      trainled(i, l1, l2);
    }
  }

  if (angle > dest) {

    for (int i = angle; i >= dest; i--) {
      angle = i;
      obj.write(angle);
      obj2.write(angle);
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
  Serial.print(Data.Pair5);
  Serial.print(", ");
  Serial.println(Data.Pair6);
}