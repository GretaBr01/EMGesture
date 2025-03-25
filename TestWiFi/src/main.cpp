#include <Arduino.h>
#include <WiFiNINA.h>

const char* ssid = "RaspberryAP";
const char* password = "RaspberryPi3b";
const char* server_ip = "192.168.5.1";
const int port = 6789;
WiFiClient client;

const int TEMPO = 1000 * 1000; //tempo del test in milli secondi: 1ks
//const int TEMPO = 10 * 1000; //tempo del test in milli secondi: 10s
const int Bps = 10000; 
int PACKETS_PER_SECOND[7] = {100, 50, 40, 25, 20, 5, 4};

const int size = sizeof(PACKETS_PER_SECOND) / sizeof(PACKETS_PER_SECOND[0]);

int BYTES_PER_PACKETS[size]; // 6x16bit + 8x12bit = 24 byte per campione (ADC + IMU)
int INTERVALS[size];  // Array per gli intervalli in millisecondi
int NUM_PACCHTTI[size]; //nuemro pacchetti da inviare in TEMPO ms
int k=0;

unsigned long totalBytesSent = 0;
int packetsSent = 0;

unsigned long sendStart;
unsigned long sendEnd;
long waitTime;
unsigned long sendDuration;

void setup(){
  for(int i=0; i< size; i++){
    BYTES_PER_PACKETS[i] = Bps / PACKETS_PER_SECOND[i];
    INTERVALS[i] = 1000 / PACKETS_PER_SECOND[i];
    NUM_PACCHTTI[i] = PACKETS_PER_SECOND[i] * TEMPO / 1000;
  }

  //connessione al WiFi  
  while (WiFi.begin(ssid, password) != WL_CONNECTED) {
    Serial.print("--");
    delay(3000);
  } 
  delay(3000);
}


void loop(){
  
  if (k >= size) {
    k=0;
    delay(5000);   
  }
  
  while (!client.connect(server_ip, port)) {
    Serial.println(".");
    delay(3000);
  } 
  
  int packet_size = BYTES_PER_PACKETS[k];
  uint8_t buffer[packet_size];

  // Riempie il pacchetto con valori casuali
  for (int i = 0; i < packet_size; i++) {
    buffer[i] = random(0, 256);
  }
  
  int num_pacchetti = 0;
  //inviare i pacchetti per TEMPO ms
  while( num_pacchetti < NUM_PACCHTTI[k] && client.connected()){
    sendStart = millis();
    int byte_inviati = client.write(buffer, packet_size);
    sendEnd = millis();    
    num_pacchetti ++;
    sendDuration = sendEnd - sendStart;
    waitTime = INTERVALS[k] - sendDuration;

    if (waitTime > 0) {
      delay(waitTime);
    }

    if(WiFi.status() != WL_CONNECTED){
      
      while (WiFi.begin(ssid, password) != WL_CONNECTED) {
        Serial.print("--");
        delay(3000);
      } 
      
    }
  }

  // Aspetta che il server chiuda la connessione
  while (client.connected()) {
    delay(100);
  }

  k++;
}