# EMGesture - International IMS Student Contest
## AcquisizioneDati_queue_wifi
Firmware Arduino rp2040 connect per campionare segnali IMU 204Hz (accelerometro e giroscopio), 4 canali ADC 1kHz.
Connessione TCP via WiFi, invia pacchetti composti da 26 blocchi |timestamp|gyr|acc| e 125 blocchi |timestamp|adc0|adc1|adc2|adc3|.
timestamp 4B,
gyr 6B,
acc 6B,
adc? 12 bit

Per la programmazione utilizzato picoSDK earlephilhower

## server_AcquisizioneDati_queue_wifi.py
Server TCP per ricezione pacchetti con wifi. 
Salva i dati in file csv, suddivide i pacchetti che hanno schema 26 blocchi |timestamp|gyr|acc| e 125 blocchi |timestamp|adc0|adc1|adc2|adc3|.

## TestWiFi
Test della connessione TCP del modulo WiFiNINA, calcolo velocità media in Kbps.

## letturaIMU
Lettura dati di accelerometro e giroscopio del sensore LSM6DSOX.

## letturaIMU_SpeedTest
Test velocità di lettura FIFO del IMU

## server.py
Server TCP che riceve pacchetti di dati, misura la velocità di trasmissione in Kbps, e salva i risultati in un file CSV.


# TO DO LIST
## Creazione Dataset
- script python che salva in file csv il timestamp di pressione tasto da tastiera, verrà utilizzato per classificare i gesti in fase di creazione dataset. Si pensa di confrontare i timestamp ricevuti dai sample e il timestamp di questo script per etichettare i sample raccolti.
- Server NTP per sincronizzare timestamp Raspberry con timestamp arduino. (Il Raspberry sarà il nostro server NTP)
- verificare dimensionamento Queue tra i due core di arduino

## Creazione NN
- test di esecuzione di una NN su Raspberry per avere un'idea di quanti neuroni e livelli possiamo implementare senza sovraccaricare al massimo Raspberry
- analisi dati raccolti (correlazione, feature significative...)
- modello NN più adeguato
- addestramento e validazione del modello