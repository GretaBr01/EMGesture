#include <Arduino.h>
#include "LSM6DSOXSensor.h"
#include <WiFiNINA.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/util/queue.h"
/**
 * AGGIUNGERE CRC
 */

#define LED_PIN LED_BUILTIN
int status = WL_IDLE_STATUS;

/* ADC */
#define MASK_12bit 0x0FFF
#define N_ADC_CHANNELS_ENABLE 4
#define N_SAMPLES_ADC 1000  //dipende quanti secndi tollerare per la perdita di connessine (il numero di campioni va moltiplicato per 2 [queue, vect])

#define N_ADC_SAMPLE_PACKET 58  // 87/1000 = 0.087 -> imu: 18/208 = 0.08653
volatile uint8_t current_channel = 0;

/* IMU */
#define SR 208.0f // frequency acc, gyr -- maggiore di 417Hz un timestamp ogni tot campioni acc,gyr
#define N_SAMPLE_IMU 208  //dipende quanti secndi tollerare per la perdita di connessine (il numero di campioni va moltiplicato per 2 [queue, vect] + 512/3 sample in IMU_FIFO)
#define WTM_LV N_SAMPLE_IMU*3 // FIFO_Watermark_Level, NUM_SAMPLE*3 -> Timestamp | giroscopio | accelerometro
#define N_IMU_SAMPLE_PACKET 12 //numero di campioni per canale che si metterà in un pacchetto: 18/208 = 0.08653
LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);

#define START_END_DELIMITER "zutss"
#define bit_START_END_DELIMITER 40
int pacchetti_inviati=0;

/* | START_DELIMITER | num_pkt_send | payload | END_DELIMITER | */
/* [5 byte start] [4 byte ID][N_IMU_SAMPLE_PACKET * (4+6+6)][N_ADC_SAMPLE_PACKET * (4+6)][5 byte end]*/
#define PACKET_SIZE (bit_START_END_DELIMITER+32+(32+6*16)*N_IMU_SAMPLE_PACKET+(32+4*12)*N_ADC_SAMPLE_PACKET+bit_START_END_DELIMITER)/8

/* Queque */
typedef struct{
  uint8_t timestamp[4]; // actual timestamp resolution value: t_actual[s] = 1/(40000 ⋅ (1 + 0.0015 ⋅ FREQ_FINE)), 
  uint8_t gyr[6];
  uint8_t acc[6];
} imu_sample;

typedef struct {
  uint32_t timestamp; // micros()
  uint16_t adc[N_ADC_CHANNELS_ENABLE];
} adc_sample;

queue_t imu_queue;
queue_t adc_queue;

/* creazione pacchetto */
typedef struct {
  uint32_t timestamp; // micros()
  uint8_t adc[6]; // 12 bit per sample adc
} adc_sample_packet;

typedef struct {
  imu_sample imu_block[N_IMU_SAMPLE_PACKET];
  adc_sample_packet adc_block[N_ADC_SAMPLE_PACKET];
} packet;

imu_sample vector_imu[N_SAMPLE_IMU];  
adc_sample_packet vector_adc[N_SAMPLES_ADC]; 

volatile bool sample_imu_ready[3] = {false, false, false};
volatile uint16_t n_sample_fifo=0;

/* WiFi */
// const char* ssid = "AndroidAP_3318_12";//"RaspberryAP";
// const char* password = "a9e1c9899c3a";//"RaspberryPi3b";
// const char* server_ip = "192.168.42.74";//"192.168.5.1";
const char* ssid = "RaspberryAP";
const char* password = "RaspberryPi3b";
const char* server_ip = "192.168.5.1";
const int port = 6789;
WiFiClient client;

int indice_imu=0;
int indice_adc=0;

int count_imu_data=0;
int count_adc_data=0;


void core1_setup();
void core1_loop();

void core1_main() {
  core1_setup();
  while (true) {
    core1_loop();
  }
}

/*******************************************************************
 *  CORE 0
 * 
 *  - ricezione dati dla Core1, immagazzina dati in array
 *  - creazione e invio pacchetto tramite connessione TCP via WiFi
 * 
 *******************************************************************/
void setup() {
  
  Serial.begin(115200);
  while (!Serial){
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }

  if (WiFi.status() == WL_NO_MODULE)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    while (true);

    digitalWrite(LED_PIN, LOW);
    delay(500);
  } 

  while (status != WL_CONNECTED){
    status = WiFi.begin(ssid, password);

    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }

  while (!client.connect(server_ip, port)) { 
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  queue_init(&imu_queue, sizeof(imu_sample), N_SAMPLE_IMU);
  queue_init(&adc_queue, sizeof(adc_sample), N_SAMPLES_ADC);
  
  multicore_launch_core1(core1_main);
}

unsigned long Time_pkt;
const unsigned long TIMEOUT_MS = 2000;  // massimo tempo timout invio pacchetto
uint32_t timeout_count=0;

void loop(){
  imu_sample imu_data;
  adc_sample adc_data;

  /* prelevo dati queue IMU - accelerometro, giroscopio, timestamp */
  if (count_imu_data < N_SAMPLE_IMU || queue_is_full(&imu_queue)) {
    if (queue_try_remove(&imu_queue, &imu_data)) {
      vector_imu[indice_imu]=imu_data;
      indice_imu = (indice_imu + 1) % N_SAMPLE_IMU;
      count_imu_data = min(count_imu_data + 1, N_SAMPLE_IMU);
    }
  }
  
  /* prelevo dati queue ADC - 4 canali adc, timestamp */
  if (count_adc_data < N_SAMPLES_ADC || queue_is_full(&adc_queue)) {
    if (queue_try_remove(&adc_queue, &adc_data)) {      
      adc_sample_packet adc_data_p; // Packing: 4x 12bit → 6 byte
      adc_data_p.timestamp=adc_data.timestamp;
      adc_data_p.adc[0] = adc_data.adc[0] & 0xFF;
      adc_data_p.adc[1] = ((adc_data.adc[0] >> 8) & 0x0F) | ((adc_data.adc[1] & 0x0F) << 4);
      adc_data_p.adc[2] = (adc_data.adc[1] >> 4) & 0xFF;
      adc_data_p.adc[3] = adc_data.adc[2] & 0xFF;
      adc_data_p.adc[4] = ((adc_data.adc[2] >> 8) & 0x0F) | ((adc_data.adc[3] & 0x0F) << 4);
      adc_data_p.adc[5] = (adc_data.adc[3] >> 4) & 0xFF;

      vector_adc[indice_adc]=adc_data_p;
      indice_adc = (indice_adc + 1) % N_SAMPLES_ADC;
      count_adc_data = min(count_adc_data + 1, N_SAMPLES_ADC);
    }
  }

  /* Create Packet */
  if(count_imu_data >= N_IMU_SAMPLE_PACKET && count_adc_data >= N_ADC_SAMPLE_PACKET){ 
    int start_imu = (indice_imu - count_imu_data + N_SAMPLE_IMU) % N_SAMPLE_IMU;
    int start_adc = (indice_adc - count_adc_data + N_SAMPLES_ADC) % N_SAMPLES_ADC;

    /* stampa timestamp primo sample imu nel pacchetto*/
    // adc_sample_packet& first_adc = vector_adc[start_adc];  // primo sample del pacchetto
    // imu_sample& first_imu = vector_imu[start_imu];
    // Serial.printf("\nADC timestamp: %lu\n", first_adc.timestamp);
    // Serial.printf("IMU timestamp: %lu\n", (first_imu.timestamp[3]<<24|first_imu.timestamp[2]<<16|first_imu.timestamp[1]<<8|first_imu.timestamp[0])); 

    uint8_t buffer[PACKET_SIZE];
    int offset = 0;

    // START DELIMITER
    for (int i = 0; i < 5; ++i)
      buffer[offset++] = START_END_DELIMITER[i];

    // Numero pacchetto
    buffer[offset++] = pacchetti_inviati & 0xFF;
    buffer[offset++] = (pacchetti_inviati >> 8) & 0xFF;
    buffer[offset++] = (pacchetti_inviati >> 16) & 0xFF;
    buffer[offset++] = (pacchetti_inviati >> 24) & 0xFF;

    for (int i = 0; i < N_IMU_SAMPLE_PACKET; ++i) {
      int index = (start_imu + i) % N_SAMPLE_IMU;
      memcpy(buffer + offset, vector_imu[index].timestamp, 4);
      offset += 4;
      memcpy(buffer + offset, vector_imu[index].gyr, 6);
      offset += 6;
      memcpy(buffer + offset, vector_imu[index].acc, 6);
      offset += 6;
    }


    for (int i = 0; i < N_ADC_SAMPLE_PACKET; ++i) {
      int index = (start_adc + i) % N_SAMPLES_ADC;

      uint32_t t = vector_adc[index].timestamp;
      buffer[offset++] = (t >> 0) & 0xFF;
      buffer[offset++] = (t >> 8) & 0xFF;
      buffer[offset++] = (t >> 16) & 0xFF;
      buffer[offset++] = (t >> 24) & 0xFF;

      memcpy(buffer + offset, vector_adc[index].adc, 6);
      offset += 6;
    }

    // END DELIMITER
    for (int i = 0; i < 5; ++i)
      buffer[offset++] = START_END_DELIMITER[i];

    /* Send packet */
    int byte_inviati=0;
    while(byte_inviati < PACKET_SIZE){
      if(client.connected()){
        byte_inviati = client.write(buffer, PACKET_SIZE);
        Serial.printf("%d, byte inviati %d\n", pacchetti_inviati, byte_inviati); 
      }else{
        client.stop();
        Serial.printf("client non connesso\n");
        client.connect(server_ip, port);
      }  
    }
    pacchetti_inviati++;
    Time_pkt=millis();

    count_adc_data=max(count_adc_data-N_ADC_SAMPLE_PACKET,0);
    count_imu_data=max(count_imu_data-N_IMU_SAMPLE_PACKET,0);
  }

  if (pacchetti_inviati>100000){
    Serial.printf("Stop per test, inviati %d pacchetti", pacchetti_inviati);
    while(true);
  }

  if(millis()-Time_pkt > TIMEOUT_MS){
    timeout_count++;
    Serial.println("INVIO PACCHETTI FERMO");
    if(!client.connected()){
      client.stop();
      Serial.printf("client non connesso\n");
      client.connect(server_ip, port);
    } else{
      client.stop();
      Serial.printf("client connesso, chiudo e riapro connessione nel caso errore server\n");
      delay(500);
      client.connect(server_ip, port);
    }
    if(timeout_count>2){
      multicore_reset_core1();
      delay(100);
      multicore_launch_core1(core1_main);
    }
    Time_pkt=millis();
    Serial.printf("FIFO_IMU = %d, queue_imu %d, vect_imu %d\n", n_sample_fifo, queue_get_level(&imu_queue), count_imu_data);
    Serial.printf("queue_adc %d, vect_adc %d\n\n", queue_get_level(&adc_queue), count_adc_data);
  }
  
}


/*******************************************************************
 *  CORE 1
 * 
 *  - sincronizzazione clock con NTP
 *  - acquisizione dati da IMU e ADC (4 canali)
 *  - invio dati al Core1 tramite queue
 * 
 *******************************************************************/

/* interrupt ADC */
static adc_sample sample_adc; 
void adc_interrupt_handler() { 
  if (current_channel == 0) {
    memset(&sample_adc, 0, sizeof(sample_adc));
    sample_adc.timestamp = micros();  // Timestamp solo al primo campione
  }
  sample_adc.adc[current_channel] = adc_fifo_get();

  if (current_channel >= N_ADC_CHANNELS_ENABLE-1) {  // tutti i canali sono stati letti 
    if (!queue_is_full(&adc_queue)){
      queue_add_blocking(&adc_queue, &sample_adc);
    }
  }

  current_channel = (current_channel + 1) % N_ADC_CHANNELS_ENABLE;
  adc_select_input(current_channel);
}

void core1_setup() {
  pinMode(LED_PIN, OUTPUT); 
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);

  /* IMU */
  Wire.begin();  
  Wire.setClock(100000);

  do {
    lsm6dsoxSensor.begin();
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  } while (lsm6dsoxSensor.Enable_X() != LSM6DSOX_OK || lsm6dsoxSensor.Enable_G() != LSM6DSOX_OK);

  uint8_t id;
  do {
    lsm6dsoxSensor.ReadID(&id);
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  } while (id != LSM6DSOX_ID);

  lsm6dsoxSensor.Set_X_FS(4);    // Accelerometro: ±4G
  lsm6dsoxSensor.Set_G_FS(2000); // Giroscopio: ±2000 dps

  lsm6dsoxSensor.Set_X_ODR(SR); 
  lsm6dsoxSensor.Set_G_ODR(SR);

  lsm6dsoxSensor.Set_FIFO_X_BDR(SR);
  lsm6dsoxSensor.Set_FIFO_G_BDR(SR);

  lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE); // Svuota la FIFO prima di iniziare

  lsm6dsoxSensor.Set_Timestamp_Status(1);
  lsm6dsoxSensor.Set_FIFO_Timestamp_Decimation(1);

  /* ADC */
  adc_init();
  adc_gpio_init(26); // pin A0
  adc_gpio_init(27); // pin A1
  adc_gpio_init(28); // pin A2
  adc_gpio_init(29); // pin A3

  adc_set_clkdiv(48000.0f/N_ADC_CHANNELS_ENABLE); //frequenza di 1kHz per canale
  adc_fifo_setup(true, false, 1, false, false);
  adc_irq_set_enabled(true);

  irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_interrupt_handler);
  irq_set_enabled(ADC_IRQ_FIFO, true);

  lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE); // Abilita modalità continua FIFO del IMU (salva campioni Timestamp, Gyr, Acc)
  adc_run(true);
}

uint32_t imu_last_sample_time = 0;
uint16_t prev_numSample = 0;
uint32_t imu_watchdog_timer = 0;
const uint32_t imu_timeout_ms = 700;  // timeout per il riavvio


void core1_loop() {

  uint16_t numSample=0;
  lsm6dsoxSensor.Get_FIFO_Num_Samples(&numSample);
  n_sample_fifo=numSample;

  if (numSample > 3) {

    imu_sample sample_imu;
    bool sample_in_queue=false;
    uint8_t fifo_data[7];

    for(int ns=0; ns<3 &&  numSample>0; ns++){
      lsm6dsoxSensor.Get_FIFO_Sample(fifo_data, 1);

      uint16_t id_sample = fifo_data[0] >> 3;
      if(id_sample == 4){
        memcpy(sample_imu.timestamp, &fifo_data[1], 4);
        sample_imu_ready[0]=true;
      }else if (id_sample == 1){  // gyro
        memcpy(sample_imu.gyr, &fifo_data[1], 6);
        sample_imu_ready[1]=true;
      }
      else if (id_sample == 2){ // acc
        memcpy(sample_imu.acc, &fifo_data[1], 6);
        sample_imu_ready[2]=true;
      }

      lsm6dsoxSensor.Get_FIFO_Num_Samples(&numSample);
      n_sample_fifo=numSample;
    }

    
    if (sample_imu_ready[0] && sample_imu_ready[1] && sample_imu_ready[2]) {
      if (!queue_is_full(&imu_queue)){
        queue_add_blocking(&imu_queue, &sample_imu);  // put sample completo nella coda  
      }   
      sample_imu_ready[0]=false;
      sample_imu_ready[1]=false;
      sample_imu_ready[2]=false;
      memset(&sample_imu, 0, sizeof(sample_imu));
      imu_last_sample_time = millis(); // aggiorna ultimo tempo valido
    }else{
      /* Se non ci sono i tre campioni in ordine resetto la FIFO */
      lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE);
      delay(1);
      lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE);
    }
  }

  /* Se non riceve nuovi campioni resetto lsm6dsoxSensor (Acc, Gyr) */
  if (millis() - imu_last_sample_time > imu_timeout_ms) {
    Serial.println("\n ----------------- IMU RESET ----------------");
    adc_run(false);
    digitalWrite(LED_PIN, HIGH);
    
    lsm6dsoxSensor.Disable_G();
    lsm6dsoxSensor.Disable_X();
    lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE);

    delay(3);

    lsm6dsoxSensor.Enable_G();
    lsm6dsoxSensor.Enable_X(); 
    lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE);
    adc_run(true);

    lsm6dsoxSensor.Get_FIFO_Num_Samples(&numSample);
    n_sample_fifo=numSample;
    imu_last_sample_time = millis();

    digitalWrite(LED_PIN, LOW);
  }
}
