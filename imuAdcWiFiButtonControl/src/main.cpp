/**
 * il primo campione del primo pacchetto delle serie temporiali successive alla prima potrebbe essere un campione da buttare
 */
#include <Arduino.h>

#include "LSM6DSOXSensor.h"
#include <WiFiNINA.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/util/queue.h"

#define LED_PIN D4
static int status = WL_IDLE_STATUS;

/* WiFi */
const char* ssid = "RaspberryAP";
const char* password = "RaspberryPi3b";
const char* server_ip = "192.168.5.1";
const int port = 6789;
static WiFiClient client;


#define QUEUE_THRESHOLD_PCT 90

/* ADC */
#define N_ADC_CHANNELS_ENABLE 4
#define N_SAMPLES_ADC_QUEUE 3016  //size of 52 packet
#define N_SAMPLES_ADC_RING N_SAMPLES_ADC_QUEUE 
#define N_SAMPLES_ADC_PACKET 58
#define THRESHOLD_ADC_QUEUE (N_SAMPLES_ADC_QUEUE * QUEUE_THRESHOLD_PCT /100)
static uint8_t current_adc_channel = 0;

typedef struct {
  uint32_t timestamp;
  uint16_t adc[N_ADC_CHANNELS_ENABLE];
} adc_sample;

/* IMU */
#define SR 208.0f // frequency acc, gyr
#define N_SAMPLES_IMU_QUEUE 624 //52 packet
#define N_SAMPLES_IMU_RING N_SAMPLES_IMU_QUEUE 
#define N_SAMPLES_IMU_PACKET 12
#define THRESHOLD_IMU_QUEUE (N_SAMPLES_IMU_QUEUE * QUEUE_THRESHOLD_PCT /100)
static LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);

#define IMU_FIFO_READ_PACKETS  3
static uint8_t fifo_data[7*(IMU_FIFO_READ_PACKETS *2 - 1)];
static uint16_t num_samples_fifo=0;

typedef struct{
  uint8_t timestamp[4]; // actual timestamp resolution value: t_actual[s] = 1/(40000 ⋅ (1 + 0.0015 ⋅ FREQ_FINE)), 
  uint8_t gyr[6];
  uint8_t acc[6];
} imu_sample;



/* creazione pacchetto */
typedef struct {
  uint32_t timestamp;
  uint8_t adc[6]; // sample adc 12 bit
} adc_sample_packet;

typedef struct {
  imu_sample imu_block[N_SAMPLES_IMU_PACKET];
  adc_sample_packet adc_block[N_SAMPLES_ADC_PACKET];
} packet;

/* | PACKAGE NUMBER | PAYLOAD | ~PACKAGE NUMBER - 4 byte | 
*
*  sizeof(PACKAGE NUMBER) = 4 byte            
*  sizeof(PAYLOAD) = [(4+6+6)*N_SAMPLES_IMU_PACKET + (4+6)*N_SAMPLES_ADC_PACKET] byte
*  sizeof(~PACKAGE NUMBER) = 4 byte
*/
static uint32_t num_pkt_inviati=0;
static uint32_t num_pkt_negato=0;
#define PACKET_SIZE (32+(32+6*16)*N_SAMPLES_IMU_PACKET+(32+4*12)*N_SAMPLES_ADC_PACKET+32)/8
static uint8_t packet_buffer[PACKET_SIZE];
static int offset = 0;


/* Queque */
static queue_t imu_queue;
static queue_t adc_queue;

/* buffer ring core0*/
static imu_sample imu_ring_core0[N_SAMPLES_IMU_RING]; 
static int imu_ring_core0_index=0;
static int imu_data_core0_count=0;

static adc_sample_packet adc_ring_core0[N_SAMPLES_ADC_RING]; 
static int adc_ring_core0_index=0;
static int adc_data_core0_count=0;

/* buffer ring core1*/
#define ADC_RING_CORE1_SIZE 900
static adc_sample adc_ring_core1[ADC_RING_CORE1_SIZE];
static uint8_t adc_ring_core1_head = 0;
static uint8_t adc_ring_core1_tail = 0;

const int RATIO_FREQ= round(1000/208); //ADC and IMU frequency ratio


/*STATE MACHINE CORE 0*/
typedef enum {
  STATE_CORE0_INIT = 0,
  STATE_CORE0_GET_IMU,
  STATE_CORE0_GET_ADC,
  STATE_CORE0_CREATE_PACKET,
  STATE_CORE0_SEND_PACKET,
  STATE_CORE0_PREPARING_STOP,
  STATE_CORE0_IDLE
} core0_state_t;

/*STATE MACHINE CORE 0*/
typedef enum {
  STATE_CORE1_INIT = 0,
  STATE_CORE1_READING_IMU,
  STATE_CORE1_READING_ADC,
  STATE_CORE1_STOP,
  STATE_CORE1_IDLE
} core1_state_t;

static core0_state_t state_core0 = STATE_CORE0_IDLE;
static core1_state_t state_core1 = STATE_CORE1_IDLE;
static bool core0_stop=true;


/* variabili di supporto */
static int adc_level_queue=0;
static int imu_level_queue=0;
static int byte_inviati=0;
static int temp_head = 0;
static int cont = 0;


/* button */
#define BUTTON_GPIO D8
static bool btn_interrupt=false;

static unsigned long  btn_time = 0;
#define DEBOUNCE_DELAY_MS 800
bool btn_enabled = true;

void button_callback(uint gpio, uint32_t events) {
  if (gpio == BUTTON_GPIO && events & GPIO_IRQ_EDGE_FALL && btn_enabled) {
    btn_enabled = false;
    btn_time=millis();
    btn_interrupt = true;
  }
}

void changeStateCore0(core0_state_t new_state){
  if(!btn_interrupt){
    state_core0 = new_state;
    return;
  }

  btn_interrupt=false;
  switch (state_core0){
    case STATE_CORE0_IDLE:
      state_core0 = STATE_CORE0_INIT;
    break;

    case STATE_CORE0_INIT:
    case STATE_CORE0_GET_IMU:
    case STATE_CORE0_GET_ADC:
    case STATE_CORE0_CREATE_PACKET:
    case STATE_CORE0_SEND_PACKET:
      state_core0 = STATE_CORE0_PREPARING_STOP;
    break;  

    default:
    break;
  }
  
}   

void setup_button_interrupt() {
  gpio_init(BUTTON_GPIO);
  gpio_set_dir(BUTTON_GPIO, GPIO_IN);
  gpio_pull_up(BUTTON_GPIO);

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
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  
  for(int i=0;i<5;i++){
    gpio_put(LED_PIN, HIGH);
    delay(50);
    gpio_put(LED_PIN, LOW);
    delay(50);
  }

  if (WiFi.status() == WL_NO_MODULE){
    gpio_put(LED_PIN, HIGH);
    delay(500);
    while (true);
  } 

  while (status != WL_CONNECTED){
    status = WiFi.begin(ssid, password);
    gpio_put(LED_PIN, HIGH);
    delay(300);
    gpio_put(LED_PIN, LOW);
    delay(300);
  }

  while (!client.connect(server_ip, port)) { 
    gpio_put(LED_PIN, HIGH);
    delay(100);
    gpio_put(LED_PIN, LOW);
    delay(100);
  }
  
  queue_init(&imu_queue, sizeof(imu_sample), N_SAMPLES_IMU_QUEUE);
  queue_init(&adc_queue, sizeof(adc_sample), N_SAMPLES_ADC_QUEUE);
  
  multicore_launch_core1(core1_main);
  setup_button_interrupt();
  state_core0 = STATE_CORE0_IDLE;
}

void loop(){
  unsigned long  now = millis();
  if (!btn_enabled && (now - btn_time > DEBOUNCE_DELAY_MS)) {
    btn_enabled = true;
  }
  switch (state_core0){
    case  STATE_CORE0_INIT: 
      core0_stop = false;  

      imu_data_core0_count=0;
      imu_ring_core0_index=0;

      adc_data_core0_count=0;
      adc_ring_core0_index=0;

      gpio_put(LED_PIN, HIGH);

      multicore_fifo_push_blocking(STATE_CORE1_INIT);
      changeStateCore0(STATE_CORE0_GET_IMU);   
      break;

    case STATE_CORE0_GET_IMU:
      /* get sample IMU queue - timestamp, gyro, acc*/
      imu_level_queue = queue_get_level(&imu_queue);
      while (imu_level_queue > 0) {
        if (imu_data_core0_count >= N_SAMPLES_IMU_RING && imu_level_queue < THRESHOLD_IMU_QUEUE) {
            break;
        }

        imu_sample imu_data;
        queue_remove_blocking(&imu_queue, &imu_data);

        imu_ring_core0[imu_ring_core0_index] = imu_data;
        imu_ring_core0_index = (imu_ring_core0_index + 1) % N_SAMPLES_IMU_RING;

        if (imu_data_core0_count < N_SAMPLES_IMU_RING) {
            imu_data_core0_count++;
        }
        imu_level_queue--;
      }

      changeStateCore0(STATE_CORE0_GET_ADC);    
      break;

    case STATE_CORE0_GET_ADC:
      /* get sample ADC queue  - timestamp, adc0, adc1, adc2, adc3 */
      adc_level_queue = queue_get_level(&adc_queue);
      while (adc_level_queue > 0) {
        if (adc_data_core0_count >= N_SAMPLES_ADC_RING && adc_level_queue < THRESHOLD_ADC_QUEUE) {
          break;
        }

        adc_sample adc_data;
        queue_remove_blocking(&adc_queue, &adc_data);

        adc_sample_packet* p = &adc_ring_core0[adc_ring_core0_index];
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

        adc_ring_core0_index = (adc_ring_core0_index + 1) % N_SAMPLES_ADC_RING;

        if (adc_data_core0_count < N_SAMPLES_ADC_RING) {
            adc_data_core0_count++;
        }
        adc_level_queue--;
      }

      changeStateCore0(STATE_CORE0_CREATE_PACKET);
      break;
    
    case  STATE_CORE0_CREATE_PACKET:
      /* Create Packet */
      if(imu_data_core0_count >= N_SAMPLES_IMU_PACKET && adc_data_core0_count >= N_SAMPLES_ADC_PACKET){ 
        int start_imu = (imu_ring_core0_index - imu_data_core0_count + N_SAMPLES_IMU_RING) % N_SAMPLES_IMU_RING;
        int start_adc = (adc_ring_core0_index - adc_data_core0_count + N_SAMPLES_ADC_RING) % N_SAMPLES_ADC_RING;

        adc_data_core0_count-=N_SAMPLES_ADC_PACKET;
        imu_data_core0_count-=N_SAMPLES_IMU_PACKET;

        offset = 0;

        /* Num_pkt */
        packet_buffer[offset++] = num_pkt_inviati & 0xFF;
        packet_buffer[offset++] = (num_pkt_inviati >> 8) & 0xFF;
        packet_buffer[offset++] = (num_pkt_inviati >> 16) & 0xFF;
        packet_buffer[offset++] = (num_pkt_inviati >> 24) & 0xFF;

        /* imu samples */
        for (int i = 0; i < N_SAMPLES_IMU_PACKET; i++) {
          int index = (start_imu + i) % N_SAMPLES_IMU_RING;
          memcpy(packet_buffer + offset, imu_ring_core0[index].timestamp, 4);
          offset += 4;
          memcpy(packet_buffer + offset, imu_ring_core0[index].gyr, 6);
          offset += 6;
          memcpy(packet_buffer + offset, imu_ring_core0[index].acc, 6);
          offset += 6;
        }

        /* adc samples */
        for (int i = 0; i < N_SAMPLES_ADC_PACKET; i++) {
          int index = (start_adc + i) % N_SAMPLES_ADC_RING;

          uint32_t t = adc_ring_core0[index].timestamp;
          packet_buffer[offset++] = (t >> 0) & 0xFF;
          packet_buffer[offset++] = (t >> 8) & 0xFF;
          packet_buffer[offset++] = (t >> 16) & 0xFF;
          packet_buffer[offset++] = (t >> 24) & 0xFF;

          memcpy(packet_buffer + offset, adc_ring_core0[index].adc, 6);
          offset += 6;
        }

        /* ~Num_pkt */
        num_pkt_negato = ~num_pkt_inviati;
        packet_buffer[offset++] = num_pkt_negato & 0xFF;
        packet_buffer[offset++] = (num_pkt_negato >> 8) & 0xFF;
        packet_buffer[offset++] = (num_pkt_negato >> 16) & 0xFF;
        packet_buffer[offset++] = (num_pkt_negato >> 24) & 0xFF;

        changeStateCore0(STATE_CORE0_SEND_PACKET);
      }else{   
        if(core0_stop){
          changeStateCore0(STATE_CORE0_PREPARING_STOP);
        }else{
          changeStateCore0(STATE_CORE0_GET_IMU);
          delay(5); 
        }
      }
      break;
    case STATE_CORE0_SEND_PACKET:
      /* Send packet */
      byte_inviati=0;
      while(byte_inviati < PACKET_SIZE){
        if(client.connected()){
          byte_inviati = client.write(packet_buffer, PACKET_SIZE);
        }else{
          client.stop();
          gpio_put(LED_PIN, LOW);
          delay(100);
          gpio_put(LED_PIN, HIGH);
          client.connect(server_ip, port);
        }  
      }
      num_pkt_inviati++;

      if(core0_stop){
        changeStateCore0(STATE_CORE0_PREPARING_STOP);
      }else{
        changeStateCore0(STATE_CORE0_GET_IMU);
      }

      break;

    case STATE_CORE0_PREPARING_STOP:
      core0_stop = true;
      multicore_fifo_push_blocking(STATE_CORE1_STOP);

      imu_level_queue = queue_get_level(&imu_queue);
      adc_level_queue = queue_get_level(&adc_queue);

      if(imu_level_queue > 0){
        changeStateCore0(STATE_CORE0_GET_IMU); 
      }else if(adc_level_queue > 0){
        changeStateCore0(STATE_CORE0_GET_ADC); 
      }else if(imu_data_core0_count >= N_SAMPLES_IMU_PACKET && adc_data_core0_count >= N_SAMPLES_ADC_PACKET){
        changeStateCore0(STATE_CORE0_CREATE_PACKET);
      }else{
        changeStateCore0(STATE_CORE0_IDLE); 
        gpio_put(LED_PIN, LOW);  
      }

      break;

    case STATE_CORE0_IDLE:
      changeStateCore0(STATE_CORE0_IDLE); 
      break;
    
    default:
      break;
  }

}


/*******************************************************************
 *  CORE 1
 *  - data acquisition from IMU and ADC (4 channels)
 *  - data sending to Core0 via queue
 * 
 *******************************************************************/


/* interrupt ADC */
static adc_sample sample_adc; 
void adc_interrupt_handler() { 
  if (current_adc_channel == 0) {
    sample_adc.timestamp = micros();
  }
  sample_adc.adc[current_adc_channel] = adc_fifo_get();

  if (current_adc_channel == N_ADC_CHANNELS_ENABLE-1) { 
    uint8_t next_head = (adc_ring_core1_head + 1) % ADC_RING_CORE1_SIZE;

    adc_ring_core1[adc_ring_core1_head] = sample_adc;
    adc_ring_core1_head = next_head;

    if (next_head == adc_ring_core1_tail) {
      adc_ring_core1_tail = (adc_ring_core1_tail + 1) % ADC_RING_CORE1_SIZE;
    }

  }

  current_adc_channel = (current_adc_channel + 1) % N_ADC_CHANNELS_ENABLE;
  adc_select_input(current_adc_channel);
}

static bool fifo_multicore_interrupt = false;
void core1_sio_irq() {
  while (multicore_fifo_rvalid()){
    state_core1 =(core1_state_t) multicore_fifo_pop_blocking(); // msg stop or msg init
    fifo_multicore_interrupt=true;
  }
  multicore_fifo_clear_irq();
}

void changeStateCore1(core1_state_t new_state){
  if(!fifo_multicore_interrupt){
    state_core1 = new_state;
  }else{
    // set state_core1 in core1_sio_irq()
    fifo_multicore_interrupt=false;  //reset interrupt
  }   
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

  lsm6dsoxSensor.Set_X_FS(4);    // Acc: ±4G
  lsm6dsoxSensor.Set_G_FS(2000); // Gyro: ±2000 dps

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

  adc_set_clkdiv(48000.0f/N_ADC_CHANNELS_ENABLE); //frequency channel 1kHz, 4 channels
  adc_fifo_setup(true, false, 1, false, false);
  adc_irq_set_enabled(true);

  irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_interrupt_handler);
  irq_set_enabled(ADC_IRQ_FIFO, true);

  state_core1 = STATE_CORE1_IDLE;
}


void core1_loop() {
  switch (state_core1){
  case STATE_CORE1_INIT:
    adc_ring_core1_head=0;
    adc_ring_core1_tail=0;
    lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE);
    adc_run(true);

    changeStateCore1(STATE_CORE1_READING_IMU);
    break;

  case STATE_CORE1_READING_IMU:
    lsm6dsoxSensor.Get_FIFO_Num_Samples(&num_samples_fifo);    

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

    changeStateCore1(STATE_CORE1_READING_ADC);
    break;

  case STATE_CORE1_READING_ADC:
    temp_head = adc_ring_core1_head;
    
    cont=0;
    while (adc_ring_core1_tail != temp_head && cont<RATIO_FREQ) {
      if(queue_try_add(&adc_queue, &adc_ring_core1[adc_ring_core1_tail])){
        adc_ring_core1_tail = (adc_ring_core1_tail + 1) % ADC_RING_CORE1_SIZE;
        cont++;
      }else{
        break;
      }
    }

    changeStateCore1(STATE_CORE1_READING_IMU);
    break;

  case STATE_CORE1_STOP:
    lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE);
    adc_run(false);
    changeStateCore1(STATE_CORE1_IDLE);
    break;

  case STATE_CORE1_IDLE:
    break;
  
  default:
    break;
  }
}