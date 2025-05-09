import os
import pandas as pd
import matplotlib.pyplot as plt

file_path = os.path.join("Dataset", "dataset", "imu_dataset.csv")
df = pd.read_csv(file_path)

df.plot(x='time_rel',y='gyr_y', label='gyr_y', linewidth=0.5)
plt.show()