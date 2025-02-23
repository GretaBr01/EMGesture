#include <Arduino.h>
#include <WiFiNINA.h>



const char* ssid = "AndroidAP_3318_12";
const char* password = "12345678";
const char* server_ip = "192.168.144.74";
const int port = 6789;
 
WiFiClient client;


const int PACKETS_PER_SECOND = 20;  // Numero di pacchetti al secondo
const int SAMPLES_PER_PACKET = 1000/PACKETS_PER_SECOND;  // Campioni per pacchetto (considerando 1k campioni al secondo)
const int BYTES_PER_SAMPLE = 24;  // 6x16bit + 8x12bit = 24 byte per campione (ADC + IMU)
//const int PACKET_SIZE = SAMPLES_PER_PACKET * BYTES_PER_SAMPLE;  // Dimensione totale del pacchetto

const int PACKET_SIZE = 2200;
uint8_t buffer[PACKET_SIZE];
unsigned long lastSendTime = 0; 

unsigned long totalBytesSent = 0;
unsigned long startTime;
int packetsSent = 0;

void printResults(unsigned long testDuration);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.print("Connessione a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    WiFi.begin(ssid, password);
    Serial.print(".");
  }
  Serial.println("\nConnesso al WiFi!");
    
  if (client.connect(server_ip, port)) {
    Serial.println("Connesso al server TCP!");
  } else {
    Serial.println("Connessione fallita. Ritento tra 5 secondi...");
    delay(5000);
  }

  startTime = millis();
}

void loop() {
  unsigned long elapsedTime = millis() - startTime;
  if (elapsedTime >= 10000) {  // Test di 10 secondi
    printResults(elapsedTime);
    while (1);  // Ferma il test
  }

  unsigned long sendStart = millis();
  client.write(buffer, PACKET_SIZE);
  unsigned long sendEnd = millis();

  totalBytesSent += PACKET_SIZE;
  packetsSent++;

  Serial.print("Pacchetto inviato in ");
  Serial.print(sendEnd - sendStart);
  Serial.println(" ms");
  delay(1000 / PACKETS_PER_SECOND);
}

void printResults(unsigned long testDuration) {
  float durationSec = testDuration / 1000.0;
  float speedKbps = (totalBytesSent * 8) / (durationSec * 1000); 
  
  Serial.println("\nTest di Performance WiFi Completato!");
  Serial.print("Durata: "); Serial.print(durationSec); Serial.println(" s");
  Serial.print("Pacchetti inviati: "); Serial.println(packetsSent);
  Serial.print("Dati trasmessi: "); Serial.print(totalBytesSent); Serial.println(" byte");
  Serial.print("Velocità media: "); Serial.print(speedKbps); Serial.println(" Kbps");
}
