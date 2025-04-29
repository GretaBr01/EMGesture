#include <Arduino.h>

#include "LSM6DSOXSensor.h"
#include <WiFiNINA.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/util/queue.h"

#define LED_PIN LED_BUILTIN
static int status = WL_IDLE_STATUS;

#define QUEUE_THRESHOLD_PCT 90

/* ADC */
#define N_ADC_CHANNELS_ENABLE 4
#define N_SAMPLES_ADC_QUEUE 3016  //52 pacchetti
#define N_SAMPLES_ADC_RING N_SAMPLES_ADC_QUEUE 
#define N_SAMPLES_ADC_PACKET 58
#define THRESHOLD_ADC_QUEUE (N_SAMPLES_ADC_QUEUE * QUEUE_THRESHOLD_PCT /100)
static uint8_t current_channel = 0;

volatile int overflow_adc_ring_core1=0;

/* IMU */
#define SR 208.0f // frequency acc, gyr -- maggiore di 417Hz un timestamp ogni tot campioni acc,gyr
#define N_SAMPLES_IMU_QUEUE 624 //52 pacchetti
#define N_SAMPLES_IMU_RING N_SAMPLES_IMU_QUEUE 
#define N_SAMPLES_IMU_PACKET 12
#define THRESHOLD_IMU_QUEUE (N_SAMPLES_IMU_QUEUE * QUEUE_THRESHOLD_PCT /100)
static LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);

#define IMU_FIFO_READ_PACKETS  3
static uint8_t fifo_data[7*(IMU_FIFO_READ_PACKETS *2 - 1)];
static uint16_t num_samples_fifo=0;

static uint32_t num_pkt_inviati=0;
static uint32_t num_pkt_negato=0;

/* | NUMERO PACCHETTO | PAYLOAD | NUMERO PACCHETTO NEGATO - 4 byte | 
*
*  sizeof(NUMERO PACCHETTO) = 4 byte            
*  sizeof(PAYLOAD) = [(4+6+6)*N_SAMPLES_IMU_PACKET + (4+6)*N_SAMPLES_ADC_PACKET] byte
*  sizeof(NUMERO PACCHETTO NEGATO) = 4 byte
*/
#define PACKET_SIZE (32+(32+6*16)*N_SAMPLES_IMU_PACKET+(32+4*12)*N_SAMPLES_ADC_PACKET+32)/8

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

static queue_t imu_queue;
static queue_t adc_queue;
static uint16_t adc_queue_max_level=0;
static uint16_t imu_queue_max_level=0;

/* creazione pacchetto */
typedef struct {
  uint32_t timestamp; // micros()
  uint8_t adc[6]; // 12 bit per sample adc
} adc_sample_packet;

typedef struct {
  imu_sample imu_block[N_SAMPLES_IMU_PACKET];
  adc_sample_packet adc_block[N_SAMPLES_ADC_PACKET];
} packet;

static imu_sample imu_ring_core0[N_SAMPLES_IMU_RING];  
static adc_sample_packet adc_ring_core0[N_SAMPLES_ADC_RING]; 

volatile uint16_t n_sample_fifo=0;

/* WiFi */
const char* ssid = "RaspberryAP";
const char* password = "RaspberryPi3b";
const char* server_ip = "192.168.5.1";
const int port = 6789;
static WiFiClient client;

static int imu_ring_index=0;
static int adc_ring_index=0;

static int imu_data_count=0;
static int adc_data_count=0;


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

  if (WiFi.status() == WL_NO_MODULE){
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    while (true);
  } 

  while (status != WL_CONNECTED){
    status = WiFi.begin(ssid, password);
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(300);
  }

  while (!client.connect(server_ip, port)) { 
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  queue_init(&imu_queue, sizeof(imu_sample), N_SAMPLES_IMU_QUEUE);
  queue_init(&adc_queue, sizeof(adc_sample), N_SAMPLES_ADC_QUEUE);
  
  multicore_launch_core1(core1_main);
}

static unsigned long time_invio_pkt;
const unsigned long TIMEOUT_MS = 2000;  // massimo tempo timout invio pacchetto
static uint32_t timeout_count=0;

void loop(){

  /* get sample IMU queue - timestamp, gyro, acc*/
  int level_queue = queue_get_level(&imu_queue);
  if (level_queue > imu_queue_max_level) {
    imu_queue_max_level = level_queue;
  }
  while (level_queue > 0) {
    if (imu_data_count >= N_SAMPLES_IMU_RING && level_queue < THRESHOLD_IMU_QUEUE) {
        break;
    }

    imu_sample imu_data;
    queue_remove_blocking(&imu_queue, &imu_data);

    imu_ring_core0[imu_ring_index] = imu_data;
    imu_ring_index = (imu_ring_index + 1) % N_SAMPLES_IMU_RING;

    if (imu_data_count < N_SAMPLES_IMU_RING) {
        imu_data_count++;
    }
    level_queue--;
  }
  
  /* get sample ADC queue  - timestamp, adc0, adc1, adc2, adc3 */
  level_queue = queue_get_level(&adc_queue);
  if (level_queue > adc_queue_max_level) {
    adc_queue_max_level = level_queue;
  }
  while (level_queue > 0) {
    if (adc_data_count >= N_SAMPLES_ADC_RING && level_queue < THRESHOLD_ADC_QUEUE) {
      break;
    }

    adc_sample adc_data;
    queue_remove_blocking(&adc_queue, &adc_data);

    adc_sample_packet* p = &adc_ring_core0[adc_ring_index];
    p->timestamp = adc_data.timestamp;

    uint16_t a0 = adc_data.adc[0];
    uint16_t a1 = adc_data.adc[1];
    uint16_t a2 = adc_data.adc[2];
    uint16_t a3 = adc_data.adc[3];

    // Packing 4 x 12 bit → 6 byte
    p->adc[0] = a0 & 0xFF;
    p->adc[1] = ((a0 >> 8) & 0x0F) | ((a1 & 0x0F) << 4);
    p->adc[2] = (a1 >> 4) & 0xFF;
    p->adc[3] = a2 & 0xFF;
    p->adc[4] = ((a2 >> 8) & 0x0F) | ((a3 & 0x0F) << 4);
    p->adc[5] = (a3 >> 4) & 0xFF;

    adc_ring_index = (adc_ring_index + 1) % N_SAMPLES_ADC_RING;

    if (adc_data_count < N_SAMPLES_ADC_RING) {
        adc_data_count++;
    }
    level_queue--;
  }

  /* Create Packet */
  if(imu_data_count >= N_SAMPLES_IMU_PACKET && adc_data_count >= N_SAMPLES_ADC_PACKET){ 
    int start_imu = (imu_ring_index - imu_data_count + N_SAMPLES_IMU_RING) % N_SAMPLES_IMU_RING;
    int start_adc = (adc_ring_index - adc_data_count + N_SAMPLES_ADC_RING) % N_SAMPLES_ADC_RING;

    adc_data_count-=N_SAMPLES_ADC_PACKET;
    imu_data_count-=N_SAMPLES_IMU_PACKET;

    uint8_t buffer[PACKET_SIZE];
    int offset = 0;

    /* Num_pkt */
    buffer[offset++] = num_pkt_inviati & 0xFF;
    buffer[offset++] = (num_pkt_inviati >> 8) & 0xFF;
    buffer[offset++] = (num_pkt_inviati >> 16) & 0xFF;
    buffer[offset++] = (num_pkt_inviati >> 24) & 0xFF;

    /* imu samples */
    for (int i = 0; i < N_SAMPLES_IMU_PACKET; i++) {
      int index = (start_imu + i) % N_SAMPLES_IMU_RING;
      memcpy(buffer + offset, imu_ring_core0[index].timestamp, 4);
      offset += 4;
      memcpy(buffer + offset, imu_ring_core0[index].gyr, 6);
      offset += 6;
      memcpy(buffer + offset, imu_ring_core0[index].acc, 6);
      offset += 6;
    }

    /* adc samples */
    for (int i = 0; i < N_SAMPLES_ADC_PACKET; i++) {
      int index = (start_adc + i) % N_SAMPLES_ADC_RING;

      uint32_t t = adc_ring_core0[index].timestamp;
      buffer[offset++] = (t >> 0) & 0xFF;
      buffer[offset++] = (t >> 8) & 0xFF;
      buffer[offset++] = (t >> 16) & 0xFF;
      buffer[offset++] = (t >> 24) & 0xFF;

      memcpy(buffer + offset, adc_ring_core0[index].adc, 6);
      offset += 6;
    }

    /* ~Num_pkt */
    num_pkt_negato = ~num_pkt_inviati;
    buffer[offset++] = num_pkt_negato & 0xFF;
    buffer[offset++] = (num_pkt_negato >> 8) & 0xFF;
    buffer[offset++] = (num_pkt_negato >> 16) & 0xFF;
    buffer[offset++] = (num_pkt_negato >> 24) & 0xFF;
   
    /* Send packet */
    int byte_inviati=0;
    while(byte_inviati < PACKET_SIZE){
      if(client.connected()){
        byte_inviati = client.write(buffer, PACKET_SIZE);
        Serial.printf("%d, byte inviati %d\n", num_pkt_inviati, byte_inviati); 
      }else{
        client.stop();
        Serial.printf("client non connesso\n");
        client.connect(server_ip, port);
      }  
    }

    num_pkt_inviati++;
    time_invio_pkt=millis();

    if (num_pkt_inviati>1000){
      Serial.printf("Stop per test, inviati %d pacchetti\n", num_pkt_inviati);
      Serial.printf("Max level queue: \n imu_queue: %d\nadc_queue: %d\n", imu_queue_max_level, adc_queue_max_level);
      Serial.printf("num overflow ring core 1 adc: %d\n", overflow_adc_ring_core1);
      while(true);
    }

  }else{
    delay(5);
  }

  if(millis()-time_invio_pkt > TIMEOUT_MS){
    timeout_count++;
    Serial.println("INVIO PACCHETTI FERMO");
    if(!client.connected()){
      client.stop();
      Serial.printf("client non connesso\n");
      client.connect(server_ip, port);
    }else{
      Serial.printf("test: invio pacchetto vuoto\n");
      uint8_t buffer[PACKET_SIZE];
      int offset = 0;
  
      /* Num_pkt */
      buffer[offset++] = num_pkt_inviati & 0xFF;
      buffer[offset++] = (num_pkt_inviati >> 8) & 0xFF;
      buffer[offset++] = (num_pkt_inviati >> 16) & 0xFF;
      buffer[offset++] = (num_pkt_inviati >> 24) & 0xFF;
      int payload=PACKET_SIZE-(sizeof(num_pkt_inviati)*2);
      memset(&buffer[offset], 0, payload);
      offset+=payload;
      /* ~Num_pkt */
      num_pkt_negato = ~num_pkt_inviati;
      buffer[offset++] = num_pkt_negato & 0xFF;
      buffer[offset++] = (num_pkt_negato >> 8) & 0xFF;
      buffer[offset++] = (num_pkt_negato >> 16) & 0xFF;
      buffer[offset++] = (num_pkt_negato >> 24) & 0xFF;

      int byte_inviati = client.write(buffer, PACKET_SIZE);
      Serial.printf("%d, byte inviati %d\n", num_pkt_inviati, byte_inviati); 
      if(byte_inviati>0){
        num_pkt_inviati++;
      }
    }

    time_invio_pkt=millis();
    Serial.printf("FIFO_IMU = %d, queue_imu %d, vect_imu %d\n", n_sample_fifo, queue_get_level(&imu_queue), imu_data_count);
    Serial.printf("queue_adc %d, vect_adc %d\n", queue_get_level(&adc_queue), adc_data_count);
    Serial.printf("Max level queue: \n imu_queue: %d\nadc_queue: %d\n", imu_queue_max_level, adc_queue_max_level);
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

#define ADC_RING_CORE1_SIZE 900
static adc_sample adc_ring_core1[ADC_RING_CORE1_SIZE];
static uint8_t adc_ring_head = 0;
static uint8_t adc_ring_tail = 0;

const int RATIO_FREQ= round(1000/208); //rapporto frequenza ADC e IMU


/* interrupt ADC */
static adc_sample sample_adc; 
void adc_interrupt_handler() { 
  if (current_channel == 0) {
    sample_adc.timestamp = micros();  // Timestamp solo al primo campione
  }
  sample_adc.adc[current_channel] = adc_fifo_get();

  if (current_channel == N_ADC_CHANNELS_ENABLE-1) {  // tutti i canali sono stati letti 
    uint8_t next_head = (adc_ring_head + 1) % ADC_RING_CORE1_SIZE;

    if (next_head != adc_ring_tail) {
      adc_ring_core1[adc_ring_head] = sample_adc;
      adc_ring_head = next_head;
    } else {
      adc_ring_core1[adc_ring_head] = sample_adc;
      adc_ring_head = next_head;
      overflow_adc_ring_core1++;
    }

  }

  current_channel = (current_channel + 1) % N_ADC_CHANNELS_ENABLE;
  adc_select_input(current_channel);
}

void core1_setup() {
  /* IMU */
  Wire.begin();  // Clock frequency 100000

  do {
    lsm6dsoxSensor.begin();
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
  } while (lsm6dsoxSensor.Enable_X() != LSM6DSOX_OK || lsm6dsoxSensor.Enable_G() != LSM6DSOX_OK);

  lsm6dsoxSensor.Set_X_FS(4);    // Accelerometro: ±4G
  lsm6dsoxSensor.Set_G_FS(2000); // Giroscopio: ±2000 dps

  lsm6dsoxSensor.Set_X_ODR(SR); 
  lsm6dsoxSensor.Set_G_ODR(SR);

  lsm6dsoxSensor.Set_FIFO_X_BDR(SR);
  lsm6dsoxSensor.Set_FIFO_G_BDR(SR);

  lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE);

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

void core1_loop() {

  lsm6dsoxSensor.Get_FIFO_Num_Samples(&num_samples_fifo);
  n_sample_fifo=num_samples_fifo;
  

  if (num_samples_fifo > 3) {
    lsm6dsoxSensor.Get_FIFO_Sample(fifo_data, IMU_FIFO_READ_PACKETS);
 
    int start = 0;
    switch ( (fifo_data[start*7] >> 3) ) {
      case 0x04: // TIMESTAMP  |TIMESTAMP|GYRO|ACC|...|...|
        break;
      case 0x01: // GYRO       |GYRO|ACC|TIMESTAMP|...|...|
        if( num_samples_fifo >= 5 ) {
          lsm6dsoxSensor.Get_FIFO_Sample(&fifo_data[7*3], 2);          
          start = 2;
        }else{
          start=-1;
        }
        break;
      case 0x02: // ACC        |ACC|TIMESTAMP|GYRO|...|...|
        if( num_samples_fifo >= 4 ) {
          lsm6dsoxSensor.Get_FIFO_Sample(&fifo_data[7*3], 1);          
          start = 1;
        }else{
          start=-1;
        }
        break;
      default:
        break;
    }

    if(start!=-1){
      imu_sample sample_imu;
      bool timestamp_ok = false;
      bool gyro_ok = false;
      bool acc_ok = false;

      for(int i=start; i<(start+3); i++){
        switch ( (fifo_data[i*7] >> 3) ) {
          case 0x04: // TIMESTAMP
            memcpy(sample_imu.timestamp, &fifo_data[i*7+1], 4);
            timestamp_ok = true;
            break;
          case 0x01: // GYRO
            memcpy(sample_imu.gyr, &fifo_data[i*7+1], 6);
            gyro_ok = true;
            break;
          case 0x02: // ACC
            memcpy(sample_imu.acc, &fifo_data[i*7+1], 6);
            acc_ok = true;
            break;
          default:
            break;
        }
      }
	  
      if (timestamp_ok && gyro_ok && acc_ok) {
        queue_add_blocking(&imu_queue, &sample_imu);
      }

    }
  }

  int temp_head = adc_ring_head;
  
  int i=0;
  while (adc_ring_tail != temp_head && i<RATIO_FREQ) {
    if(queue_try_add(&adc_queue, &adc_ring_core1[adc_ring_tail])){
      adc_ring_tail = (adc_ring_tail + 1) % ADC_RING_CORE1_SIZE;
      i++;
    }else{
      break;  //queue piena
    }
  }
}