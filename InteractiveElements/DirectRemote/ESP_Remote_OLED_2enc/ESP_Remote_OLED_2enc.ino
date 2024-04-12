#include <ESP32RotaryEncoder.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const uint8_t DI_ENCODER_A   = 36; //39
const uint8_t DI_ENCODER_B   = 39; //36
const int8_t  DI_ENCODER_SW  = 14;

const uint8_t DI2_ENCODER_A   = 17;  //16
const uint8_t DI2_ENCODER_B   = 16; //17
const int8_t  DI2_ENCODER_SW  = 13;

const uint8_t LONG_PRESS = 2000;


#include <esp_now.h>
#include <WiFi.h>

#define SlaveCnt 5

volatile bool turnedRightFlag = false;
volatile bool turnedLeftFlag = false;
volatile bool turnedRightFlag2 = false;
volatile bool turnedLeftFlag2 = false;


// Structure to hold peer information
typedef struct {
  uint8_t peer_addr[6];
} SlaveInfo;

SlaveInfo slaves[SlaveCnt];

uint8_t broadcastAddresses[][6] = {
  {0xEC, 0x62, 0x60, 0x76, 0xD0, 0x04}, // Light Test ESP
  {0x40, 0x22, 0xD8, 0x77, 0x15, 0xA4}, // Train Test ESP
  {0xD8, 0xBC, 0x38, 0x75, 0x13, 0x1C}, // Crossing (X)
  {0xD8, 0xBC, 0x38, 0x75, 0x22, 0x54},  // 4 Way (4W)
  {0xD8, 0xBC, 0x38, 0x75, 0x22, 0x64}, // 3 Way (3W)

  
};

// Define the structure that will hold the data to be sent over ESP-NOW
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
int BufferClear;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
 char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

RotaryEncoder encoder( DI_ENCODER_A, DI_ENCODER_B, DI_ENCODER_SW);
RotaryEncoder encoder2( DI2_ENCODER_A, DI2_ENCODER_B, DI2_ENCODER_SW);

int values[8] = {0, 0, 0, 0, 0, 0, 0, 5};
int mult[8] = {1,1,1,1,10,10,10,10};
int maxval[8] = {10,10,10,10,100,100,100,100};
int minval[8] = {0,0,0,0,0,0,0,0};
int store[8] = {0,0,0,0,0,0,0,0};
int temp[8] = {0,0,0,0,0,0,0,0};
String LEDstates[11] = {"Off","Stop","Yellow","Go","Right","Left","R+Fwd","L+Fwd","R+Stop","L+Stop","All"};
String Controlstates[8] = {"Four Way Light","Four Way Light","Three Way Light","Three Way Light","Train Crossing", "Train Crossing","Auxilliary", "Auxilliary"};

int currentPair = 0;
volatile int passPair=0;



void setup() {
Serial.begin(9600);
 
  WiFi.mode(WIFI_STA);
 
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

    for (int i = 0; i < SlaveCnt; i++) {
    memcpy(peerInfo.peer_addr, broadcastAddresses[i], 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }

    encoder.setEncoderType( EncoderType::HAS_PULLUP );
    encoder.setBoundaries( 1, 10, true );
    encoder.onTurned( &knobCallback );
    encoder.onPressed( &buttonCallback );
    encoder2.setEncoderType( EncoderType::HAS_PULLUP );
    encoder2.setBoundaries( 0, 7, true );
    encoder2.onTurned( &knobCallback2 );
    encoder2.onPressed( &buttonCallback2 );
    encoder.begin();
    encoder2.begin();

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));}

    display.display();

}}

void loop() {

  	if( turnedRightFlag ){
		  turnedRight();
      esp_err_t result = esp_now_send(0, (uint8_t *) &Data, sizeof(DataStruct));
     // SerialDisp();
}

	  if( turnedLeftFlag ){
		  turnedLeft();
      esp_err_t result = esp_now_send(0, (uint8_t *) &Data, sizeof(DataStruct));
     // SerialDisp();
    }
    if(currentPair != passPair){
      
      currentPair=passPair;
      runDisplay();
      //Serial.println(passPair);
    }

    
  delay(10);




}


void knobCallback( long value )
{
	if( turnedRightFlag || turnedLeftFlag )
		return;

	switch( value )
	{
		case 1:
	  		turnedRightFlag = true;
		break;

		case 9:
	  		turnedLeftFlag = true;
		break;
	}

	encoder.setEncoderValue( 0 );
}

void knobCallback2( long value )
{
  

	if( turnedRightFlag2 || turnedLeftFlag2 )
		return;

passPair = value;
//Serial.println(passPair);
}

void buttonCallback( unsigned long duration ){

ButtonPress();
	
//LEDindicate();
runDisplay();
}
void buttonCallback2( unsigned long duration ){

ButtonPress2();
	
//LEDindicate();
runDisplay();
}

void turnedRight()
{
    int ctrl = currentPair;
    values[ctrl] += mult[ctrl];
    
    if(values[ctrl] >= maxval[ctrl]){values[ctrl] = maxval[ctrl];}

    Data.Pair1 = values[0];
    Data.Pair2 = values[1];
    Data.Pair3 = values[2];
    Data.Pair4 = values[3];
    Data.Pair5 = values[4];
    Data.Pair6 = values[5];
    Data.Pair7 = values[6];
    Data.Pair8 = values[7];
    runDisplay();
	turnedRightFlag = false;
}


void turnedLeft()
{
  int ctrl = currentPair;
    values[ctrl] -= mult[ctrl];
    if(values[ctrl] <= minval[ctrl]){values[ctrl] = minval[ctrl];}
    runDisplay();
    Data.Pair1 = values[0];
    Data.Pair2 = values[1];
    Data.Pair3 = values[2];
    Data.Pair4 = values[3];
    Data.Pair5 = values[4];
    Data.Pair6 = values[5];
    Data.Pair7 = values[6];
    Data.Pair8 = values[7];
	turnedLeftFlag = false;
}


void ButtonPress(){
  store[currentPair]=values[currentPair];
  
}

void ButtonPress2(){
  static bool flag;
  if(flag == true){
    Data.Pair1 = values[0];
    Data.Pair2 = values[1];
    Data.Pair3 = values[2];
    Data.Pair4 = values[3];
    Data.Pair5 = values[4];
    Data.Pair6 = values[5];
    Data.Pair7 = values[6];
    Data.Pair8 = values[7];
    flag = false;
  }
  else{
    flag = true;

    Data.Pair1 = store[0];
    Data.Pair2 = store[1];
    Data.Pair3 = store[2];
    Data.Pair4 = store[3];
    Data.Pair5 = store[4];
    Data.Pair6 = store[5];
    Data.Pair7 = store[6];
    Data.Pair8 = store[7];}
esp_err_t result = esp_now_send(0, (uint8_t *) &Data, sizeof(DataStruct));

}


void runDisplay(){
    
    display.clearDisplay();

    displayArray();
    displaycursor();
    display.display();}

    void displayArray() {
  display.setTextSize(1);      
  display.setTextColor(SSD1306_WHITE); 
  display.setCursor(0, 0);
  String topline;
  String line;
  topline += "Ctrl: " + Controlstates[currentPair];
  display.print(topline);
  display.setCursor(0, 16);
  int i;
  if(currentPair<2){
    i=0;
    line += String(LEDstates[values[i]]) + " " + String(LEDstates[values[i+1]]);  // Concatenate the number with a space
    display.print(line);
  }
  else if(currentPair<4){
    i=2;
    line += String(LEDstates[values[i]]) + " " + String(LEDstates[values[i+1]]);  // Concatenate the number with a space
    display.print(line);
  }
  else if(currentPair<6){
    i=4;
    line += String(values[i]) + " " + String(values[i+1]);  // Concatenate the number with a space
    display.print(line);
  }
  else{
    i=6;
    line += String(values[i]) + " " + String(values[i+1]);  // Concatenate the number with a space
    display.print(line);
    
}
}
void displaycursor(){
  if(currentPair%2 == 1){
    display.setCursor(64, 24);
    display.print(">>>");
  }
  else{
    display.setCursor(44, 24);
    display.print("<<<");
  }
}

void SerialDisp(){
  Serial.print("Control index");
  Serial.println(currentPair+1);
  for(int i = 0; i <8; i++){
    Serial.print(values[i]);
    Serial.print(" ");
  }
  Serial.println("");
}


int values[8] = {0, 0, 0, 0, 0, 0, 0, 5};
int mult[8] = {1,1,1,1,10,10,10,10};
int maxval[8] = {10,10,10,10,100,100,100,100};
int minval[8] = {0,0,0,0,0,0,0,0};
String LEDstates[11] = {"Off","Stop","Yellow","Go","Right","Left","R+Fwd","L+Fwd","R+Stop","L+Stop","All"};
String Controlstates[8] = {"Four Way Light","Four Way Light","Three Way Light","Three Way Light","Train Crossing", "Train Crossing","Auxilliary", "Auxilliary"};
