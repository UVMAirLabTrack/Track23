#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

#define green_pwr 120
#define yellow_pwr 1//255
#define blue_pwr 120
#define white_pwr 120
#define red_pwr 255

//[Red,Yellow,Green,White,Blue]
//LEDS controlled by array*respective pwr.  so if you want indivitual control, change powers to 1, and set the 0-255 in the
static int S0[] = {0,0,0,0,0};  //off
static int S1[] = {1,0,0,1,1}; //stop
static int S2[] = {0,1,0,0,0}; //caution
static int S3[] = {0,0,1,0,1}; //go
static int S4[] = {0,0,0,1,0}; //white light 
static int S5[] = {0,0,0,0,1}; //blue Light
static int S6[] = {0,1,0,1,0}; //opt
static int S7[] = {0,0,1,0,1}; //opt
static int S8[] = {0,0,1,1,1}; //opt
static int S9[] = {0,1,0,1,1}; //opt
static int S10[] = {1,1,1,1,1}; //opt

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

int Ctrl1 = 0;
int Ctrl2 = 0;
int Ctrl3 = 0;
int Ctrl4 = 0;

/* PCB Connections/ V1
Connector 1      Connector 2     Connector 3      Connector 4
Red 12               15              6                21 
Yellow 14             4            Linked 1           Linked 2
Green 26             18            Linked 1           Linked 2
White  25            19              8                23
Blue 27               5              7                22
*/
//Idiot ME, using 6,7,8,9,10,11 BAD IDEA!
//Good Pins  4 13 14 maybe 15 16 17 18 19 21 22 23 25 26 27 32 33
//25 flashes before first write for some reason.
/* PCB Connections/ V2
Connector 1      Connector 2     Connector 3      Connector 4
Red 12               32              16                21 
Yellow 14             4            Linked 1           Linked 2
Green 26             18              13                25
White  33            19              17                23
Blue 27               5              16                22
*/


#define redPin 12
#define yellowPin 14
#define greenPin 26
#define whitePin 33//25
#define bluePin 27



#define redPin2 32//15
#define yellowPin2 4
#define greenPin2 18
#define whitePin2 19
#define bluePin2 5


#define redPin3 13//13
#define greenPin3 15//18
#define whitePin3 17//16
#define bluePin3 16//17



#define redPin4 21
#define greenPin4 25//18
#define whitePin4 23
#define bluePin4 22





  int red;
  int yellow;
  int green;
  int white;
  int blue;

enum Color {Off, Red, Yellow, Green,White,Blue, Green_White, Green_Blue, Red_White, Red_Blue, All,Glow};
Color color;


int timer = 0;



void setup() {
  Serial.begin(115200);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(whitePin, OUTPUT);

  pinMode(redPin2, OUTPUT);
  pinMode(greenPin2, OUTPUT);
  pinMode(bluePin2, OUTPUT);
  pinMode(yellowPin2, OUTPUT);
  pinMode(whitePin2, OUTPUT);

  pinMode(redPin4, OUTPUT);
  pinMode(greenPin4, OUTPUT);
  pinMode(whitePin4, OUTPUT);
  pinMode(bluePin4, OUTPUT);

  pinMode(redPin3, OUTPUT);
  pinMode(greenPin3, OUTPUT);
  pinMode(whitePin3, OUTPUT);
  pinMode(bluePin3, OUTPUT);
  



//Esp Init Code
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;}
  esp_now_register_recv_cb(OnDataRecv);
  analogWrite(whitePin, 0); // silences the flashing white light on output 25.
 LEDinit();
}


void loop() {
if(color == Glow){
    glowfade();
}

}


void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Data, incomingData, sizeof(Data));
  /*Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Relevant Data: ");*/
  Ctrl1 = Data.Pair1;
  Ctrl2 = Data.Pair2;
  Ctrl3 = Data.Pair3;
  Ctrl4 = Data.Pair4;
  /*Serial.print(Ctrl1);
  Serial.print(", ");
  Serial.print(Ctrl2);
  Serial.print(", ");
  Serial.print(Ctrl3);
  Serial.print(", ");
  Serial.println(Ctrl4);*/

  activateLight(Ctrl4, redPin4, yellowPin2, greenPin4, whitePin4, bluePin4 );
  activateLight(Ctrl3, redPin3, yellowPin, greenPin3, whitePin3, bluePin3 );

  activateLight(Ctrl2, redPin2, yellowPin2, greenPin2, whitePin2, bluePin2);
  activateLight(Ctrl1, redPin, yellowPin, greenPin, whitePin, bluePin );

 // printDataStructure(Data);


 




}

void printDataStructure(const DataStruct &data) {
    char lines[4][50], dataArray[50];

    // Format lines with message and variables
    sprintf(lines[0], "4_way:------[ %d,  %d,  %d, %d]", data.Pair1, data.Pair2, data.Pair3, data.Pair4);
    sprintf(lines[1], "3_way:----- [ %d,  %d,  %d, %d]", data.Pair5, data.Pair6, data.Pair7, data.Pair8);
    sprintf(lines[2], "Train_Xing:-[ %d,  %d,  %d, %d]", data.Pair9, data.Pair10, data.Pair11, data.Pair12);
    sprintf(lines[3], "Aux:------- [ %d,  %d,  %d, %d]", data.Pair13, data.Pair14, data.Pair15, data.Pair16);

    // Format the array at the end
    sprintf(dataArray, "Data array: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]", 
            data.Pair1, data.Pair2, data.Pair3, data.Pair4,
            data.Pair5, data.Pair6, data.Pair7, data.Pair8,
            data.Pair9, data.Pair10, data.Pair11, data.Pair12,
            data.Pair13, data.Pair14, data.Pair15, data.Pair16);

    // Print the lines to serial
    for (int i = 0; i < 4; ++i) {
        Serial.println(lines[i]);
    }
    Serial.println(dataArray);
}

void activateLight(int ctrl ,int rpin, int ypin, int gpin, int wpin, int bpin){
  Color_Convert_int(ctrl);
  Color_state();
  setColor(rpin, ypin, gpin, wpin, bpin);
}

void setColor(int rpin, int ypin, int gpin, int wpin, int bpin) {
  analogWrite(rpin, red);
  analogWrite(gpin, green);
  analogWrite(bpin, blue);
  //analogWrite(ypin, yellow);
  digitalWrite(ypin, yellow);
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
  case 11:
  color = Glow;
}}

void Color_state(){

switch (color){
    case Off:
    red = red_pwr*S0[0];
    yellow = yellow_pwr*S0[1];
    green = green_pwr*S0[2];
    white = white_pwr*S0[3];
    blue = blue_pwr*S0[4];
    break;

  case Red:
    red = red_pwr*S1[0];
    yellow = yellow_pwr*S1[1];
    green = green_pwr*S1[2];
    white = white_pwr*S1[3];
    blue = blue_pwr*S1[4];
    break;

case Yellow:
    red = red_pwr*S2[0];
    yellow = yellow_pwr*S2[1];
    green = green_pwr*S2[2];
    white = white_pwr*S2[3];
    blue = blue_pwr*S2[4];
    break;
  
case Green:
    red = red_pwr*S3[0];
    yellow = yellow_pwr*S3[1];
    green = green_pwr*S3[2];
    white = white_pwr*S3[3];
    blue = blue_pwr*S3[4];
    break;

    case White:
    red = red_pwr*S4[0];
    yellow = yellow_pwr*S4[1];
    green = green_pwr*S4[2];
    white = white_pwr*S4[3];
    blue = blue_pwr*S4[4];
    break;

    case Blue:
    red = red_pwr*S5[0];
    yellow = yellow_pwr*S5[1];
    green = green_pwr*S5[2];
    white = white_pwr*S5[3];
    blue = blue_pwr*S5[4];
    break;

case Green_White:
    red = red_pwr*S6[0];
    yellow = yellow_pwr*S6[1];
    green = green_pwr*S6[2];
    white = white_pwr*S6[3];
    blue = blue_pwr*S6[4];
    break;

case Green_Blue:
    red = red_pwr*S7[0];
    yellow = yellow_pwr*S7[1];
    green = green_pwr*S7[2];
    white = white_pwr*S7[3];
    blue = blue_pwr*S7[4];
    break;
 
 case Red_White:
    red = red_pwr*S8[0];
    yellow = yellow_pwr*S8[1];
    green = green_pwr*S8[2];
    white = white_pwr*S8[3];
    blue = blue_pwr*S8[4];
    break;

    case Red_Blue:
    red = red_pwr*S9[0];
    yellow = yellow_pwr*S9[1];
    green = green_pwr*S9[2];
    white = white_pwr*S9[3];
    blue = blue_pwr*S9[4];
    break;
    
    case All:
    red = red_pwr*S10[0];
    yellow = yellow_pwr*S10[1];
    green = green_pwr*S10[2];
    white = white_pwr*S10[3];
    blue = blue_pwr*S10[4];
    break;

    case Glow:
    break;

default:
break;
  }
}

void LEDinit(){
  for (int j = 0; j <= 2; j++){
for(int i = 1; i <=5;i++){
  
  if(i==1){
    green = green_pwr;
    yellow = 0;
    red = 0;
    white = 0;
    blue = 0;
  }
    if(i==2){
    green = 0;
    yellow = yellow_pwr;
    red = 0;
    white = 0;
    blue = 0;
  }
    if(i==3){
    green = 0;
    yellow = 0;
    red = red_pwr;
    white = 0;
    blue = 0;
  }
    if(i==4){
    green = 0;
    yellow = 0;
    red = 0;
    white = white_pwr;
    blue = 0;
  }
    if(i==5){
    green = 0;
    yellow = 0;
    red = 0;
    white = 0;
    blue = blue_pwr;  
  }
  delay(250);
  setColor(redPin, yellowPin, greenPin, whitePin, bluePin);
  setColor(redPin3, yellowPin, greenPin3, whitePin3, bluePin3);
  setColor(redPin2, yellowPin2, greenPin2, whitePin2, bluePin2);
  setColor(redPin4, yellowPin2, greenPin4, whitePin4, bluePin4);}}

delay(250);
    green = green_pwr;
    yellow = yellow_pwr;
    red = red_pwr;
    white = white_pwr;
    blue = blue_pwr;
    
  setColor(redPin, yellowPin, greenPin, whitePin, bluePin);
  setColor(redPin3, yellowPin, greenPin3, whitePin3, bluePin3);
  setColor(redPin2, yellowPin2, greenPin2, whitePin2, bluePin2);
  setColor(redPin4, yellowPin2, greenPin4, whitePin4, bluePin4);
  delay(2000);
    green = 0;
    yellow = 0;
    red = 0;
    white = 0;
    blue = 0;
  setColor(redPin, yellowPin, greenPin, whitePin, bluePin);
  setColor(redPin3, yellowPin, greenPin3, whitePin3, bluePin3);
  setColor(redPin2, yellowPin2, greenPin2, whitePin2, bluePin2);
  setColor(redPin4, yellowPin2, greenPin4, whitePin4, bluePin4);
  }

void glowfade(){
  delay(20);
  int j;
for(int i = 0; i<255*2+1;i++) {
if(i >= 255){j = 255*2-i;}
else{j = i;}
Serial.println(j);
analogWrite(redPin,j);
analogWrite(redPin2,j);
analogWrite(redPin3,j);
analogWrite(redPin4,j);


}}
