#include <Arduino.h>
#include "LSM6DSOXSensor.h"
#include <WiFiNINA.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/util/queue.h"


#define LED_PIN LED_BUILTIN
int status = WL_IDLE_STATUS;

/* ADC */
#define MASK_12bit 0x0FFF
#define N_ADC_CHANNELS_ENABLE 4
#define N_SAMPLES_ADC 1000
#define N_ADC_SAMPLE_PACKET 125
volatile uint8_t current_channel = 0;

/* IMU */
#define SR 208.0f // frequency acc, gyr -- maggiore di 417Hz un timestamp ogni tot campioni acc,gyr
#define N_SAMPLE_IMU 128
#define WTM_LV N_SAMPLE_IMU*3 // FIFO_Watermark_Level, NUM_SAMPLE*3 -> Timestamp | giroscopio | accelerometro
#define N_IMU_SAMPLE_PACKET 26 //numero di campioni per canale che si metterà in un pacchetto -> 8pps
LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);


#define PACKET_SIZE ((32+6*16)*N_IMU_SAMPLE_PACKET+(32+4*12)*N_ADC_SAMPLE_PACKET)/8
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

volatile bool sample_ready[3] = {false, false, false}; 
volatile uint16_t n_sample_fifo=0;

/* NTP - sincronizzazione clock */
//WiFiUDP udp;
//const int timeZone = 1;  // Fuso orario (1 per CET, 0 per UTC, ecc.)
//NTPClient timeClient(udp, server_ip, 3600 * timeZone, 60000);

/* WiFi */
const char* ssid = "AndroidAP_3318_12";//"RaspberryAP";
const char* password = "a9e1c9899c3a";//"RaspberryPi3b";
const char* server_ip = "192.168.40.74";//"192.168.5.1";
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
  if (WiFi.status() == WL_NO_MODULE)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    while (true);

    digitalWrite(LED_PIN, LOW);
    delay(500);
  }

  /*String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION)
  {
    Serial.println("Please upgrade the firmware");
  }*/

  while (status != WL_CONNECTED){
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
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

void loop(){
  imu_sample imu_data;
  adc_sample adc_data;

  /* prelevo dati queue IMU - accelerometro, giroscopio, timestamp */
  if (queue_try_remove(&imu_queue, &imu_data)) {
    vector_imu[indice_imu]=imu_data;
    indice_imu = (indice_imu + 1) % N_SAMPLE_IMU;
    count_imu_data++;
  }
  
  /* prelevo dati queue ADC - 4 canali adc, timestamp */
  if (queue_try_remove(&adc_queue, &adc_data)) {
    // Packing: 4x 12bit → 6 byte
    adc_sample_packet adc_data_p;
    adc_data_p.timestamp=adc_data.timestamp;

    adc_data_p.adc[0] = adc_data.adc[0] & 0xFF;
    adc_data_p.adc[1] = ((adc_data.adc[0] >> 8) & 0x0F) | ((adc_data.adc[1] & 0x0F) << 4);
    adc_data_p.adc[2] = (adc_data.adc[1] >> 4) & 0xFF;
    adc_data_p.adc[3] = adc_data.adc[2] & 0xFF;
    adc_data_p.adc[4] = ((adc_data.adc[2] >> 8) & 0x0F) | ((adc_data.adc[3] & 0x0F) << 4);
    adc_data_p.adc[5] = (adc_data.adc[3] >> 4) & 0xFF;

    vector_adc[indice_adc]=adc_data_p;
    indice_adc = (indice_adc + 1) % N_SAMPLES_ADC;
    count_adc_data++;
  }

  if(count_imu_data >= N_IMU_SAMPLE_PACKET && count_adc_data >= N_ADC_SAMPLE_PACKET){ 
    int start_imu = (indice_imu - count_imu_data + N_SAMPLE_IMU) % N_SAMPLE_IMU;
    uint8_t buffer[PACKET_SIZE];
    int offset = 0;

    for (int i = 0; i < N_IMU_SAMPLE_PACKET; ++i) {
      memcpy(buffer + offset, vector_imu[(start_imu + i) % N_SAMPLE_IMU].timestamp, 4);
      offset += 4;
    
      memcpy(buffer + offset, vector_imu[(start_imu + i) % N_SAMPLE_IMU].gyr, 6);
      offset += 6;
    
      memcpy(buffer + offset, vector_imu[(start_imu + i) % N_SAMPLE_IMU].acc, 6);
      offset += 6;
    }

    int start_adc = (indice_adc - count_adc_data + N_SAMPLES_ADC) % N_SAMPLES_ADC;
    for (int i = 0; i < N_ADC_SAMPLE_PACKET; ++i) {
      // Timestamp ADC (4 byte)
      int32_t t = vector_adc[(start_adc + i) % N_SAMPLES_ADC].timestamp;
      buffer[offset++] = (t >> 0) & 0xFF;
      buffer[offset++] = (t >> 8) & 0xFF;
      buffer[offset++] = (t >> 16) & 0xFF;
      buffer[offset++] = (t >> 24) & 0xFF;
    
      // 6 byte packed ADC
      memcpy(buffer + offset, vector_adc[(start_adc + i) % N_SAMPLES_ADC].adc, 6);
      offset += 6;
    }

    int byte_inviati = client.write(buffer, sizeof(buffer));
    Serial.printf("byte inviati: %d, %d\n", byte_inviati, sizeof(buffer));
    count_adc_data=max(count_adc_data-N_ADC_SAMPLE_PACKET,0);
    count_imu_data=max(count_imu_data-N_IMU_SAMPLE_PACKET,0);
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
static adc_sample sample; 
void adc_interrupt_handler() { 
  if (current_channel == 0) {
    sample={0};
    sample.timestamp = micros();  // Timestamp solo al primo campione
  }
  sample.adc[current_channel] = adc_fifo_get();

  if (current_channel >= N_ADC_CHANNELS_ENABLE-1) {  // Se tutti i canali sono stati letti 
    queue_add_blocking(&adc_queue, &sample);  // Inserisci la struct nella queue
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

  lsm6dsoxSensor.Set_FIFO_Watermark_Level(1);  //soglia cambio stato della FIFO

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
const uint32_t imu_timeout_ms = 200;  // timeout per il riavvio


void core1_loop() {
  static uint32_t last_check_time = 0;

  uint16_t numSample=0;
  lsm6dsoxSensor.Get_FIFO_Num_Samples(&numSample);
  n_sample_fifo=numSample;

  if (numSample > 0) {
    imu_last_sample_time = millis(); // aggiorna ultimo tempo valido

    imu_sample sample;
    uint8_t fifo_data[7];

    while(numSample>0){
      lsm6dsoxSensor.Get_FIFO_Sample(fifo_data, 1);

      uint16_t id_sample = fifo_data[0] >> 3;
      if(id_sample == 4){
        memcpy(sample.timestamp, &fifo_data[1], 4);
        sample_ready[0]=true;
      }else if (id_sample == 1){  // gyro
        memcpy(sample.gyr, &fifo_data[1], 6);
        sample_ready[1]=true;
      }
      else if (id_sample == 2){ // acc
        memcpy(sample.acc, &fifo_data[1], 6);
        sample_ready[2]=true;
      }

      if (sample_ready[0] && sample_ready[1] && sample_ready[2]) {
        queue_add_blocking(&imu_queue, &sample);  // Inserisci il sample completo nella coda
        sample_ready[0]=false;
        sample_ready[1]=false;
        sample_ready[2]=false;
        sample={0};
      }

      lsm6dsoxSensor.Get_FIFO_Num_Samples(&numSample);
    }
  }

  if (millis() - imu_last_sample_time > imu_timeout_ms) {
    digitalWrite(LED_PIN, HIGH);
    imu_last_sample_time = millis();
    lsm6dsoxSensor.Disable_G();
    lsm6dsoxSensor.Disable_X();
    lsm6dsoxSensor.Enable_G();
    lsm6dsoxSensor.Enable_X();

    lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE);
    delay(2);
    lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE);

    digitalWrite(LED_PIN, LOW);
  }

}
