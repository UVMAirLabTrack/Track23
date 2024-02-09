#include <esp_now.h>
#include <WiFi.h>

#define SlaveCnt 5

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

void setup() {
  Serial.begin(115200);
  //Serial.begin(9600);
  Serial.setTimeout(50);
 
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
  }
  
}




void loop() {
 if (Serial.available() >= 6) {
    // Read the integers from Serial
    Data.Pair1 = Serial.parseInt();
    Data.Pair2 = Serial.parseInt();
    Data.Pair3 = Serial.parseInt();
    Data.Pair4 = Serial.parseInt();
    Data.Pair5 = Serial.parseInt();
    Data.Pair6 = Serial.parseInt();
    Data.Pair7 = Serial.parseInt();
    Data.Pair8 = Serial.parseInt();
    BufferClear = Serial.parseInt();
/*
        Serial.print("String: ");
        Serial.print(Data.Pair1);
    Serial.print(" ");
        Serial.print(Data.Pair2);
    Serial.print(" ");
        Serial.print(Data.Pair3);
    Serial.print(" ");
        Serial.print(Data.Pair4);
    Serial.print(" ");
        Serial.print(Data.Pair5);
    Serial.print(" ");
        Serial.print(Data.Pair6);
    Serial.print(" ");
        Serial.print(Data.Pair7);
    Serial.print(" ");
        Serial.println(Data.Pair8);   */


esp_err_t result = esp_now_send(0, (uint8_t *) &Data, sizeof(DataStruct));

      

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }}

}


/*//Broadcasd Addresses to the Peers
uint8_t broadcastAddress1[] = {0xEC, 0x62, 0x60, 0x76, 0xD0, 0x04}; // Light Test Esp
uint8_t broadcastAddress2[] = {0x40,0x22 ,0xD8 ,0x77 ,0x15 ,0xA4 }; //Train Test ESP
uint8_t broadcastAddress3[] = {0xD8,0xBC ,0x38 ,0x75 ,0x13 ,0x1c }; // Crossing (X)
uint8_t broadcastAddress4[] = {0xD8,0xBC ,0x38 ,0x75 ,0x22 ,0x64 };  // 3 Way  (3W)
uint8_t broadcastAddress6[] = {0xD8,0xBC ,0x38 ,0x75 ,0x22 ,0x54 }; // 4 Way (4W) */

// register first peer  
  /*memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add Light Test Peer");
    return;
  }
  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add Train Test Peer");
    return;
  }
  /// register third peer
  memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add Crossing Peer");
    return;
  }
    /// register fourth peer
  memcpy(peerInfo.peer_addr, broadcastAddress4, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add 3 Way Peer");
    return;
  }
    /// register fifth peer
  memcpy(peerInfo.peer_addr, broadcastAddress6, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add 4 Way Peer");
    return;
  }*/
