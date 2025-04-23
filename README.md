# EMGesture - International IMS Student Contest
## AcquisizioneDati_queue_wifi
Firmware Arduino rp2040 connect per campionare segnali IMU 204Hz (accelerometro e giroscopio), 4 canali ADC 1kHz.
Connessione TCP via WiFi, invia pacchetti composti da 12 blocchi |timestamp|gyr|acc| e 58 blocchi |timestamp|adc0|adc1|adc2|adc3|.
timestamp 4B,
gyr 6B,
acc 6B,
adc 12 bit

Per la programmazione utilizzato picoSDK earlephilhower


# TO DO LIST

## Creazione NN
- test di esecuzione di una NN su Raspberry per avere un'idea di quanti neuroni e livelli possiamo implementare senza sovraccaricare al massimo Raspberry
- analisi dati raccolti (correlazione, feature significative...)
- modello NN più adeguato
- addestramento e validazione del modello
