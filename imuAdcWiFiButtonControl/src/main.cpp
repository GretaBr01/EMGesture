#include <Arduino.h>
#include "LSM6DSOXSensor.h"
#include <WiFiNINA.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/util/queue.h"

#include "pico/multicore.h"

#define LED_PIN LED_BUILTIN

/* WiFi*/
int status = WL_IDLE_STATUS;
const char* ssid = "RaspberryAP";
const char* password = "RaspberryPi3b";
const char* server_ip = "192.168.5.1";
const int port = 6789;
WiFiClient client;

/* ADC */
#define MASK_12bit 0x0FFF
#define N_ADC_CHANNELS_ENABLE 4
#define N_SAMPLES_ADC_QUEUE 1000
#define N_SAMPLE_ADC_PACKET 58
static uint8_t current_adc_channel = 0;

/* IMU */
#define SR 208.0f // frequency acc, gyr
#define N_SAMPLE_IMU_QUEUE 208
#define N_SAMPLE_IMU_PACKET 12
LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);

#define IMU_FIFO_READ_PACKETS  3
static uint8_t fifo_data[7*(IMU_FIFO_READ_PACKETS *2 - 1)];
static uint16_t num_samples_fifo=0;

uint32_t num_pkt_inviati=0;
uint32_t num_pkt_negato=0;

/* | NUMERO PACCHETTO | PAYLOAD | NUMERO PACCHETTO NEGATO - 4 byte | 
*
*  sizeof(NUMERO PACCHETTO) = 4 byte            
*  sizeof(PAYLOAD) = [(4+6+6)*N_SAMPLE_IMU_PACKET + (4+6)*N_SAMPLE_ADC_PACKET] byte
*  sizeof(NUMERO PACCHETTO NEGATO) = 4 byte
*/
#define PACKET_SIZE (32+(32+6*16)*N_SAMPLE_IMU_PACKET+(32+4*12)*N_SAMPLE_ADC_PACKET+32)/8

/* Queque */
typedef struct{
  uint8_t timestamp[4]; // timestamp resolution value: t_actual[s] = 1/(40000 ⋅ (1 + 0.0015 ⋅ FREQ_FINE)), 
  uint8_t gyr[6];
  uint8_t acc[6];
} imu_sample;

typedef struct {
  uint32_t timestamp; // micros()
  uint16_t adc[N_ADC_CHANNELS_ENABLE];
} adc_sample;

queue_t imu_queue;
queue_t adc_queue;

/* pacchetto */
typedef struct {
  uint32_t timestamp; // micros()
  uint8_t adc[6]; // 12 bit per sample adc
} adc_sample_packet;

typedef struct {
  imu_sample imu_block[N_SAMPLE_IMU_PACKET];
  adc_sample_packet adc_block[N_SAMPLE_ADC_PACKET];
} packet;

imu_sample vector_imu[N_SAMPLE_IMU_QUEUE];  
adc_sample_packet vector_adc[N_SAMPLES_ADC_QUEUE]; 

static int indice_imu=0;
static int indice_adc=0;

static int count_imu_data=0;
static int count_adc_data=0;


#define BUTTON_GPIO D7
typedef enum {
  START = 0,
  RUN   = 1,
  STOP  = 2,
  WAIT = 3
} SystemState;

static SystemState state_core0 = WAIT;
static SystemState state_core1 = WAIT;

void button_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_FALL) {
      switch (state_core0){
        case WAIT:
          state_core0 = START;
        break;

        case RUN:
          state_core0 = STOP;
        break;

        case START:
        break;

        case STOP:
        break;
        
        default:
        break;
      }
    }
}

void setup_button_interrupt() {
  gpio_init(BUTTON_GPIO);
  gpio_set_dir(BUTTON_GPIO, GPIO_IN);
  gpio_pull_up(BUTTON_GPIO);  // Assumi bottone attivo LOW

  gpio_set_irq_enabled_with_callback(BUTTON_GPIO, GPIO_IRQ_EDGE_FALL, true, &button_callback);
}

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
  
  queue_init(&imu_queue, sizeof(imu_sample), N_SAMPLE_IMU_QUEUE);
  queue_init(&adc_queue, sizeof(adc_sample), N_SAMPLES_ADC_QUEUE);
  
  multicore_launch_core1(core1_main);

  setup_button_interrupt();
  Serial.println("start...");
}


void loop(){
  switch (state_core0){
    case START:
      multicore_fifo_push_blocking(START);
      state_core0=RUN;
    break;

    case RUN:
      imu_sample imu_data;
      adc_sample adc_data;

      /* get sample IMU queue - timestamp, gyro, acc*/
      if (count_imu_data < N_SAMPLE_IMU_QUEUE || queue_is_full(&imu_queue)) {
        if (queue_try_remove(&imu_queue, &imu_data)) {
          vector_imu[indice_imu]=imu_data;
          indice_imu = (indice_imu + 1) % N_SAMPLE_IMU_QUEUE;
          count_imu_data = min(count_imu_data + 1, N_SAMPLE_IMU_QUEUE);
        }
      }
      
      /* get sample ADC queue  - timestamp, adc0, adc1, adc2, adc3 */
      if (count_adc_data < N_SAMPLES_ADC_QUEUE || queue_is_full(&adc_queue)) {
        if (queue_try_remove(&adc_queue, &adc_data)) {      
          adc_sample_packet adc_data_p; // Packing: 4 x 12bit → 6 byte
          adc_data_p.timestamp=adc_data.timestamp;
          adc_data_p.adc[0] = adc_data.adc[0] & 0xFF;
          adc_data_p.adc[1] = ((adc_data.adc[0] >> 8) & 0x0F) | ((adc_data.adc[1] & 0x0F) << 4);
          adc_data_p.adc[2] = (adc_data.adc[1] >> 4) & 0xFF;
          adc_data_p.adc[3] = adc_data.adc[2] & 0xFF;
          adc_data_p.adc[4] = ((adc_data.adc[2] >> 8) & 0x0F) | ((adc_data.adc[3] & 0x0F) << 4);
          adc_data_p.adc[5] = (adc_data.adc[3] >> 4) & 0xFF;

          vector_adc[indice_adc]=adc_data_p;
          indice_adc = (indice_adc + 1) % N_SAMPLES_ADC_QUEUE;
          count_adc_data = min(count_adc_data + 1, N_SAMPLES_ADC_QUEUE);
        }
      }

      /* Create Packet */
      if(count_imu_data >= N_SAMPLE_IMU_PACKET && count_adc_data >= N_SAMPLE_ADC_PACKET){ 
        int start_imu = (indice_imu - count_imu_data + N_SAMPLE_IMU_QUEUE) % N_SAMPLE_IMU_QUEUE;
        int start_adc = (indice_adc - count_adc_data + N_SAMPLES_ADC_QUEUE) % N_SAMPLES_ADC_QUEUE;

        uint8_t buffer[PACKET_SIZE];
        int offset = 0;

        /* Num_pkt */
        buffer[offset++] = num_pkt_inviati & 0xFF;
        buffer[offset++] = (num_pkt_inviati >> 8) & 0xFF;
        buffer[offset++] = (num_pkt_inviati >> 16) & 0xFF;
        buffer[offset++] = (num_pkt_inviati >> 24) & 0xFF;

        /* imu samples */
        for (int i = 0; i < N_SAMPLE_IMU_PACKET; ++i) {
          int index = (start_imu + i) % N_SAMPLE_IMU_QUEUE;
          memcpy(buffer + offset, vector_imu[index].timestamp, 4);
          offset += 4;
          memcpy(buffer + offset, vector_imu[index].gyr, 6);
          offset += 6;
          memcpy(buffer + offset, vector_imu[index].acc, 6);
          offset += 6;
        }

        /* adc samples */
        for (int i = 0; i < N_SAMPLE_ADC_PACKET; ++i) {
          int index = (start_adc + i) % N_SAMPLES_ADC_QUEUE;

          uint32_t t = vector_adc[index].timestamp;
          buffer[offset++] = (t >> 0) & 0xFF;
          buffer[offset++] = (t >> 8) & 0xFF;
          buffer[offset++] = (t >> 16) & 0xFF;
          buffer[offset++] = (t >> 24) & 0xFF;

          memcpy(buffer + offset, vector_adc[index].adc, 6);
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

        count_adc_data=max(count_adc_data-N_SAMPLE_ADC_PACKET,0);
        count_imu_data=max(count_imu_data-N_SAMPLE_IMU_PACKET,0);
      }
    break;

    case STOP:
      multicore_fifo_push_blocking(STOP);
      //svuotare queue imu e adc e inviare sample rimanenti
      state_core0 = WAIT;
    break;

    case WAIT:
    break;

    default:
    break;
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
  if (current_adc_channel == 0) {
    sample_adc.timestamp = micros();  // Timestamp solo al primo campione
  }
  sample_adc.adc[current_adc_channel] = adc_fifo_get();

  if (current_adc_channel == N_ADC_CHANNELS_ENABLE-1) {  // tutti i canali sono stati letti 
    queue_try_add(&adc_queue, &sample_adc);
    memset(&sample_adc, 0, sizeof(sample_adc));
  }

  current_adc_channel = (current_adc_channel + 1) % N_ADC_CHANNELS_ENABLE;
  adc_select_input(current_adc_channel);
}

void core1_sio_irq() {
  // Just record the latest entry
  while (multicore_fifo_rvalid())
    state_core1 =(SystemState) multicore_fifo_pop_blocking();

  multicore_fifo_clear_irq();
}

void core1_setup() {

  /* FIFO MultiCore*/
  multicore_fifo_clear_irq();
  irq_set_exclusive_handler(SIO_FIFO_IRQ_NUM(1), core1_sio_irq);
  irq_set_enabled(SIO_FIFO_IRQ_NUM(1), true);

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
  adc_run(true);
}

void core1_loop() {
  switch (state_core1){
    case START:
      current_adc_channel=0;
      lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE);
      irq_set_enabled(ADC_IRQ_FIFO, true);

      state_core1=RUN;
    break;

    case RUN:
      lsm6dsoxSensor.Get_FIFO_Num_Samples(&num_samples_fifo);  

      if (num_samples_fifo > 3) {
        lsm6dsoxSensor.Get_FIFO_Sample(fifo_data, IMU_FIFO_READ_PACKETS);
    
        int start = -1;
        switch ( (fifo_data[start*7] >> 3) ) {
          case 0x04: // TIMESTAMP  |TIMESTAMP|GYRO|ACC|...|...|
            start=0;
            break;
          case 0x01: // GYRO       |GYRO|ACC|TIMESTAMP|...|...|
            if( num_samples_fifo >= 5 ) {
              lsm6dsoxSensor.Get_FIFO_Sample(&fifo_data[7*3], 2);          
              start = 2;
            }
            break;
          case 0x02: // ACC        |ACC|TIMESTAMP|GYRO|...|...|
            if( num_samples_fifo >= 4 ) {
              lsm6dsoxSensor.Get_FIFO_Sample(&fifo_data[7*3], 1);          
              start = 1;
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
            queue_try_add(&imu_queue, &sample_imu);
          }

        }
      }

    break;

    case STOP:
      irq_set_enabled(ADC_IRQ_FIFO, false);
      lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE);
      state_core1 = WAIT;
    break;

    case WAIT:
    break;
    
    default:
    break;
  }
  
}
