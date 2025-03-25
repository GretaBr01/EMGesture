/**
 * The LSM6DSOX embeds 3 KB of data in FIFO to store the following data:
 * • Gyroscope
 * • Accelerometer
 * • External sensors (up to four)
 * • Step counter
 * • Timestamp
 * • Temperature 
 * 
 */


   #include "LSM6DSOXSensor.h"

   #define INT_1 LSM6DSOX_INT1_PIN
   #define LED_PIN LED_BUILTIN
   #define WTM_LV 125*3  //numero di campioni (Timestamp+Gyr+Acc) che si metterà in un pacchetto -> 5pps
   #define SR 833.0f //frequenza acc, gyr
   #define FIFO_SIZE 512 // Dimensione massima della FIFO
   
   LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);
   
   uint8_t Tag; // FIFO data sensor identifier
   
   int16_t accelBuffer[WTM_LV/2][3]; 
   int16_t gyroBuffer[WTM_LV/2][3];
   uint8_t fifo[WTM_LV][7];
     
   int filledRowsAcc = 0;
   int filledRowsGyr = 0;
   
   static uint8_t wtmStatus = 0;
   uint16_t numSamples = 0; // number of samples in FIFO
   
   
   unsigned long startTime;
   
   void setup() {
     Serial.begin(115200);
     while (!Serial);
     Serial.println("inizio");
     pinMode(LED_PIN, OUTPUT);  
   
     Wire.begin();  
     Wire.setClock(400000);
   
     do {
       lsm6dsoxSensor.begin();
       digitalWrite(LED_PIN, HIGH);
       delay(500);
       digitalWrite(LED_PIN, LOW);
       delay(500);
     } while (lsm6dsoxSensor.Enable_X() != LSM6DSOX_OK || lsm6dsoxSensor.Enable_G() != LSM6DSOX_OK);
   
     uint8_t id;
     do {
       lsm6dsoxSensor.ReadID(&id);
       digitalWrite(LED_PIN, HIGH);
       delay(500);
       digitalWrite(LED_PIN, LOW);
       delay(500);
     } while (id != LSM6DSOX_ID);
   
     lsm6dsoxSensor.Set_X_FS(4);    // Accelerometro: ±4G
     lsm6dsoxSensor.Set_G_FS(2000); // Giroscopio: ±2000 dps
   
     lsm6dsoxSensor.Set_X_ODR(SR); 
     lsm6dsoxSensor.Set_G_ODR(SR);
   
     lsm6dsoxSensor.Set_FIFO_X_BDR(SR);
     lsm6dsoxSensor.Set_FIFO_G_BDR(SR);
   
     lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE); // Svuota la FIFO prima di iniziare
     delay(10);
     lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE); // Abilita modalità continua
   
     lsm6dsoxSensor.Set_FIFO_Watermark_Level(WTM_LV);  //soglia cambio stato della FIFO
   
   
     startTime=micros();
     Serial.println("START...");
   }
   
     
   void loop() {
   
     // Check if FIFO threshold level was reached.
     lsm6dsoxSensor.Get_FIFO_Watermark_Status(&wtmStatus);
   
     if(wtmStatus != 0){
       uint32_t endTime = micros();
       Serial.printf("tempo campionamento %d capioni: %lu\n", WTM_LV, (endTime-startTime));
       startTime=endTime;
       lsm6dsoxSensor.Get_FIFO_Num_Samples(&numSamples);
       Serial.println(numSamples);
         
       filledRowsAcc = 0;
       filledRowsGyr = 0;
   
       uint32_t stimeRead = micros();
       for (uint16_t i = 0; i < WTM_LV; i++) {
         lsm6dsoxSensor.Get_FIFO_Sample(fifo[i], 1); //108,240 ms
       }
       unsigned long endtimeRead = micros();
       Serial.printf("tempo lettura %d capioni: %lu\n\n", WTM_LV, (endtimeRead-stimeRead));
     }
     
   }