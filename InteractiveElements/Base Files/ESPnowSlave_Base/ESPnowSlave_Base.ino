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





void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:


//Esp Init Code
WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;}
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    Color_Convert_int(Data.Pair1);
    Color_state();
    setColor(redPin, yellowPin, greenPin, whitePin, bluePin);
    
    Color_Convert_int(Data.Pair2);
    Color_state();
    setColor(redPin2, yellowPin2, greenPin2, whitePin2, bluePin2);

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

oid setColor(int rpin, int ypin, int gpin, int wpin, int bpin) {
  analogWrite(rpin, red);
  analogWrite(gpin, green);
  analogWrite(bpin, blue);
  analogWrite(ypin, yellow);
  analogWrite(wpin, white);
}

void Color_Convert_int(int value){
switch (value){
  case 0:
  color = Off;
  break;
  case 1:
  color = Red;
  break;
    case 2:
  color = Yellow;
  break;
    case 3:
  color = Green;
  break;
    case 4:
  color = White;
  break;
    case 5:
  color = Blue;
  break;
    case 6:
  color = Green_White;
  break;
    case 7:
  color = Green_Blue;
  break;
    case 8:
  color = Red_White;
  break;
    case 9:
  color = Red_Blue;
  break;
    case 10:
  color = All;
  break;
}}

void Color_state(){

switch (color){
    case Off:
    red = 0;
    yellow = 0;
    green = 0;
    white = 0;
    blue = 0;
    break;

  case Red:
    red = red_pwr;
    yellow = 0;
    green = 0;
    white = 0;
    blue = 0;
    break;

case Yellow:
    red = 0;
    yellow = yellow_pwr;
    green = 0;
    white = 0;
    blue = 0;
    break;
  
case Green:
    red = 0;
    yellow = 0;
    green = green_pwr;
    white = 0;
    blue = 0;
    break;

    case White:
    red = 0;
    yellow = 0;
    green = 0;
    white = white_pwr;
    blue = 0;
    break;

    case Blue:
    red = 0;
    yellow = 0;
    green = 0;
    white = 0;
    blue = blue_pwr;
    break;

case Green_White:
    red = 0;
    yellow = 0;
    green = green_pwr;
    white = white_pwr;
    blue = 0;
    break;

case Green_Blue:
    red = 0;
    yellow = 0;
    green = green_pwr;
    white = 0;
    blue = blue_pwr;
    break;
 
 case Red_White:
    red = red_pwr;
    yellow = 0;
    green = 0;
    white = white_pwr;
    blue = 0;
    break;

    case Red_Blue:
    red = red_pwr;
    yellow = 0;
    green = 0;
    white = 0;
    blue = blue_pwr;
    break;
    
    case All:
        red = red_pwr;
    yellow = yellow_pwr;
    green = green_pwr;
    white = blue_pwr;
    blue = blue_pwr;
    break;
default:
break;
  }
}