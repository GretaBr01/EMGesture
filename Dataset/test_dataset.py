# test_dataset.py Script created to understand the dataset
import pandas as pd

df = pd.read_csv("dataset/adc_dataset.csv")
print("ADC dataset labels: ", df.label.unique())

df = pd.read_csv("dataset/imu_dataset.csv")
print("IMU dataset labels: ", df.label.unique())




