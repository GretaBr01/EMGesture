# EMGesture

## electrode position
![PCB](images/PCB.png)
![Elettrodi](images/electrodes_position.png)

## Gesture: 
- Left turn
- Stopping
- Right turn

![Gesture](images/gesture.png)

reference: https://channel.endu.net/ciclismo/il-codice-dei-ciclisti/

## Experimental Setup
![Experimental Setup](images/ExperimentalSetup.png)

## Sensor:
- IMU ST Microelectronics LSM6DSOXTR (Acc, Gyro): 16 bit @ 208 Sa/s
- Raspberry Pi RP2040 - ADC: 4 channel, 12 bit @ 1kSa/s

range Acc: [-4, +4]g +/-0.122 mg
sensitivity: LSM6DSOX_ACC_SENSITIVITY_FS_4G   0.122f

range Gyro: [-2000, +2000]dps +/-70mdps
sensitivity: LSM6DSOX_GYRO_SENSITIVITY_FS_2000DPS  70.000f

range ADC: [0, 4095]
