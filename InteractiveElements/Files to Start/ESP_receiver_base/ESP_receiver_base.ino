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
#define CHANNEL 1

void setup() {
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;}
  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {
  // put your main code here, to run repeatedly:

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