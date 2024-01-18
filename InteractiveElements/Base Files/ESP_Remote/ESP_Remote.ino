#include <ESP32RotaryEncoder.h>


const uint8_t DI_ENCODER_A   = 32;
const uint8_t DI_ENCODER_B   = 27;
const int8_t  DI_ENCODER_SW  = 25;
const int8_t  DO_ENCODER_VCC = 13;
const uint8_t LONG_PRESS = 2000;


#include <esp_now.h>
#include <WiFi.h>

#define SlaveCnt 5

volatile bool turnedRightFlag = false;
volatile bool turnedLeftFlag = false;


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

int values[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int mult[8] = {1, 1, 1, 10, 10,10,10};
int maxval[8] = {10,10,10,10,100,100,100,100};
int minval[8] = {0,0,0,0,0,0,0,0};
int currentPair = 0;
int stepsPerPair = 1;

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

    // Range of values to be returned by the encoder: minimum is 1, maximum is 10
    // The third argument specifies whether turning past the minimum/maximum will
    // wrap around to the other side:
    //  - true  = turn past 10, wrap to 1; turn past 1, wrap to 10
    //  - false = turn past 10, stay on 10; turn past 1, stay on 1
    encoder.setBoundaries( 1, 10, true );

    // The function specified here will be called every time the knob is turned
    // and the current value will be passed to it
    encoder.onTurned( &knobCallback );

    // The function specified here will be called every time the button is pushed and
    // the duration (in milliseconds) that the button was down will be passed to it
    encoder.onPressed( &buttonCallback );

    // This is where the inputs are configured and the interrupts get attached
    encoder.begin();

}}

void loop() {

  	if( turnedRightFlag ){
		  turnedRight();
      esp_err_t result = esp_now_send(0, (uint8_t *) &Data, sizeof(DataStruct));
}

	  if( turnedLeftFlag ){
		  turnedLeft();
      esp_err_t result = esp_now_send(0, (uint8_t *) &Data, sizeof(DataStruct));
    }


  delay(10);




}

/*void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  for (int i = 0; i < 8; ++i) {
    display.setCursor(i * 15, 10);
    display.print(values[i]);
  }

  display.display();
}*/

void knobCallback( long value )
{
  //Serial.println(value);
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

void buttonCallback( unsigned long duration ){
  if( duration > LONG_PRESS )
	{
		buttonLongPress();
	}
	else
	{
		ButtonPress();
	}
LEDindicate();

}

void turnedRight()
{
    values[currentPair] += mult[currentPair];
    
    if(values[currentPair] >= maxval[currentPair]){values[currentPair] = maxval[currentPair];}
    values[currentPair+1]=values[currentPair];
 //   values[currentPair + 1] += mult[currentPair+1];
   // if(values[currentPair+1] >= maxval[currentPair+1]){values[currentPair+1] = maxval[currentPair+1];}
    Serial.print(values[currentPair]);
    Serial.print("  ");
    Serial.println(values[currentPair+1]);
    Data.Pair1 = values[1];
    Data.Pair2 = values[2];
    Data.Pair3 = values[3];
    Data.Pair4 = values[4];
    Data.Pair5 = values[5];
    Data.Pair6 = values[6];
    Data.Pair7 = values[7];
    Data.Pair8 = values[8];
    //updateDisplay();
	turnedRightFlag = false;
}

void turnedLeft()
{
    values[currentPair] -= mult[currentPair];
    if(values[currentPair] <= minval[currentPair]){values[currentPair] = minval[currentPair];}
    values[currentPair+1]=values[currentPair];
   // values[currentPair + 1] -= mult[currentPair];
   // if(values[currentPair+1] <= minval[currentPair+1]){values[currentPair+1] = minval[currentPair+1];}
    //updateDisplay();
    Data.Pair1 = values[1];
    Data.Pair2 = values[2];
    Data.Pair3 = values[3];
    Data.Pair4 = values[4];
    Data.Pair5 = values[5];
    Data.Pair6 = values[6];
    Data.Pair7 = values[7];
    Data.Pair8 = values[8];
    Serial.print(values[currentPair]);
    Serial.print("  ");
    Serial.println(values[currentPair+1]);

	// Set this back to false so we can watch for the next move
	turnedLeftFlag = false;
}

void ButtonPress(){
  currentPair += 2;
    currentPair %= 8; // Wrap around if needed
    Serial.print("Current Pair: ");
    Serial.println(currentPair);
   // updateDisplay();
}

void buttonLongPress()
{
esp_err_t result = esp_now_send(0, (uint8_t *) &Data, sizeof(DataStruct));
}

void LEDindicate(){
    int l1 = 0;
  int l2 = 0;
    if(currentPair == 0){
      l1 = 0;
      l2 = 0;
    }
    if(currentPair == 2){
      l1 = 1;
      l2 = 0;
    }
    if(currentPair == 4){
      l1 = 0;
      l2 = 1;
    }

    if(currentPair == 6){
      l1 = 1;
      l2 = 2;
    }

    analogWrite(22,l1*255);
    analogWrite(23,l2*255);
}
