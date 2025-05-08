import os
import pandas as pd
import numpy as np

from scipy.stats import skew
from scipy.stats import kurtosis

# === CONFIGURAZIONE ===
folder = "dataset"
adc_path = f"./{folder}/emg_dataset.csv"
imu_path = f"./{folder}/imu_dataset.csv"

output_folder = "dataset_features"
output_adc_path = f"./{output_folder}/emg_features_dataset.csv"
output_imu_path = f"./{output_folder}/imu_features_dataset.csv"
output_df_path = f"./{output_folder}/features_dataset.csv"

os.makedirs(output_folder, exist_ok=True)

# === DATI SENSORE ===
SENSOR_DATA = {
    "acc": {"frequenza_campionamento": 208, "full_scale": 2**15}, # data raw
    "gyr": {"frequenza_campionamento": 208, "full_scale": 2**15}, # data raw
    "emg": {"frequenza_campionamento": 1000, "full_scale": 4095.0},
}

# === FUNZIONI STATISTICHE ===
feature_functions = {
    'mean': lambda x, **kwargs: np.mean(x),
    'std': lambda x, **kwargs: np.std(x),
    # 'c_mean': lambda x, **kwargs: np.array([np.mean(x), np.std(x)]),
    # 'c_mean': lambda x, **kwargs: complex(np.mean(x), np.std(x)),
    'min': lambda x, **kwargs: np.min(x),
    'max': lambda x, **kwargs: np.max(x),
    'q5': lambda x, **kwargs: np.percentile(x, 5),
    'q10': lambda x, **kwargs: np.percentile(x, 10),
    'q90': lambda x, **kwargs: np.percentile(x, 90),
    'q95': lambda x, **kwargs: np.percentile(x, 95),
    'energy': lambda x, delta_t, **kwargs: np.sum(x**2) * delta_t,
    'rms': lambda x, **kwargs: np.sqrt(np.mean(x**2)),
    'energy_ac': lambda x_ac, delta_t, **kwargs: np.sum(x_ac**2) * delta_t,
    'rms_ac': lambda x_ac, **kwargs: np.sqrt(np.mean(x_ac**2)), 
    'skewness': lambda x, **kwargs: skew(x),
    'kurtosis': lambda x, **kwargs: kurtosis(x)
}

excluded_features=[]
num_features=len(feature_functions)-len(excluded_features)

# === UTILITY ===
def get_sensor_type(col_name):
  for sensor in SENSOR_DATA:
    if col_name.startswith(sensor):
      return sensor
  return None  # se non corrisponde a nessun sensore


def signal_statistic(x, sensor_name, Norm=True):
  if Norm:
    x = x/SENSOR_DATA[sensor_name]["full_scale"]

  delta_t = 1.0/SENSOR_DATA[sensor_name]["frequenza_campionamento"]
  
  x_dc = np.mean(x)
  x_ac = x - x_dc
  L = len(x)

  delta_t = 1.0/SENSOR_DATA[sensor_name]["frequenza_campionamento"]

  var_stats = {
    feature: func(x=x, x_ac=x_ac, delta_t=delta_t, L=L)  # Passa tutte le variabili necessarie
    for feature, func in feature_functions.items()
    if feature not in excluded_features
  }
  return pd.Series(var_stats)

def dfFeatures(df, sensors):
  features_list = []
  for sensor in sensors:
    stats_df = (
      df.groupby(['label', 'series_id'])[sensor]
      .apply(lambda x: signal_statistic(x, sensor_name=get_sensor_type(sensor), Norm=True))
      .unstack()  # Trasforma le statistiche in colonne
      .reset_index()  # Resetta l'indice per un DataFrame piatto
    )

    # Aggiungi il nome del sensore come prefisso per le colonne delle statistiche
    stats_df = stats_df.add_prefix(f"{sensor}_")
    stats_df.rename(columns={f"{sensor}_label": "label", f"{sensor}_series_id": "series_id"}, inplace=True)
    features_list.append(stats_df)

  features_df = features_list[0]
  for other_df in features_list[1:]:
    features_df = pd.merge(features_df, other_df, on=["label", "series_id"])

  return features_df

# === IMU ===
df_imu = pd.read_csv(imu_path)

# Calcolo vettore accelerometro e giroscopio
df_imu["acc_vector"] = np.sqrt(df_imu["acc_x"]**2 + df_imu["acc_y"]**2 + df_imu["acc_z"]**2)
df_imu["gyr_vector"] = np.sqrt(df_imu["gyr_x"]**2 + df_imu["gyr_y"]**2 + df_imu["gyr_z"]**2)

df_imu_features=df_imu.drop(columns=["timestamp", "pkt_time_ns"])
sensors_imu=(df_imu_features.columns.values.tolist())
sensors_imu.remove("series_id")
sensors_imu.remove("label")
df_imu_features = dfFeatures(df_imu_features, sensors_imu)


# === ADC ===
df_adc = pd.read_csv(adc_path)
df_adc_features=df_adc.drop(columns=["timestamp", "pkt_time_ns"])
sensors_adc=(df_adc_features.columns.values.tolist())
sensors_adc.remove("series_id")
sensors_adc.remove("label")
df_adc_features = dfFeatures(df_adc_features, sensors_adc)



df_imu_features.to_csv(output_imu_path, index=False)
df_adc_features.to_csv(output_adc_path, index=False)

# === MERGE TOTALE ===
df_features = pd.merge(df_imu_features, df_adc_features, on=["label", "series_id"])
df_features.to_csv(output_df_path, index=False)

import pickle
with open("file.pk", "wb") as f:
  pickle.dump(df_features, f)

