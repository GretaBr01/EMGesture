import os
import pandas as pd
import numpy as np
from scipy.stats import skew, kurtosis
from tqdm import tqdm
from tqdm.auto import tqdm as tqdm_auto


# === PARAMETRI VARIABILI ===
window_size_adc = 125   # numero di campioni per finestra
window_size_imu = 26
use_overlap = True
overlap_ratio = 0.1 if use_overlap else 0.0  # percentuale di sovrapposizione

# === CONFIGURAZIONE ===
folder = "dataset"
adc_path = f"./{folder}/adc_dataset.csv"
imu_path = f"./{folder}/imu_dataset.csv"
output_folder = f"dataset_features_slidingWindow_{window_size_adc}"
output_adc_path = f"./{output_folder}/adc_features_dataset.csv"
output_imu_path = f"./{output_folder}/imu_features_dataset.csv"
output_df_path = f"./{output_folder}/features_dataset.csv"
os.makedirs(output_folder, exist_ok=True)



# === DATI SENSORE ===
SENSOR_DATA = {
    "acc": {"frequenza_campionamento": 208, "full_scale": 2**15},
    "gyr": {"frequenza_campionamento": 208, "full_scale": 2**15},
    "adc": {"frequenza_campionamento": 1000, "full_scale": 4095.0},
}

# === FUNZIONI STATISTICHE ===
feature_functions = {
    'mean': lambda x, **kwargs: np.mean(x),
    'std': lambda x, **kwargs: np.std(x),
    'min': lambda x, **kwargs: np.min(x),
    'max': lambda x, **kwargs: np.max(x),
    'q5': lambda x, **kwargs: np.percentile(x, 5),
    'q10': lambda x, **kwargs: np.percentile(x, 10),
    'q90': lambda x, **kwargs: np.percentile(x, 90),
    'q95': lambda x, **kwargs: np.percentile(x, 95),
    'energy': lambda x, delta_t, **kwargs: np.sum(x**2) * delta_t,
    'rms': lambda x, **kwargs: np.sqrt(np.mean(x**2)),
    'skewness': lambda x, **kwargs: skew(x),
    'kurtosis': lambda x, **kwargs: kurtosis(x)
}

excluded_features = []
num_features = len(feature_functions) - len(excluded_features)

# === UTILITY ===
def get_sensor_type(col_name):
    for sensor in SENSOR_DATA:
        if col_name.startswith(sensor):
            return sensor
    return None

def signal_statistic(x, sensor_name, Norm=True):
    if Norm:
        x = x / SENSOR_DATA[sensor_name]["full_scale"]

    delta_t = 1.0 / SENSOR_DATA[sensor_name]["frequenza_campionamento"]
    
    var_stats = {
        feature: func(x=x, delta_t=delta_t)
        for feature, func in feature_functions.items()
        if feature not in excluded_features
    }
    return pd.Series(var_stats)

def sliding_window_df(df, window_size, overlap_ratio):
    step = int(window_size * (1 - overlap_ratio))
    windowed = []

    grouped = df.groupby(["label", "series_id"])
    for (label, series_id), group in tqdm(grouped, desc="Sliding windows"):
        data = group.reset_index(drop=True)
        data = data.sort_values("timestamp").reset_index(drop=True) 

        window_number = 0 
        for start in range(0, len(data) - window_size + 1, step):
            window = data.iloc[start:start + window_size].copy()
            window["window_id"] = f"{series_id}_{window_number}"
            windowed.append(window)
            window_number+=1
    return pd.concat(windowed, ignore_index=True)

def dfFeatures(df, sensors):
    features = []
    grouped = df.groupby("window_id")
    for sensor in tqdm(sensors, desc="Extracting features"):
        stats_df = grouped[sensor].apply(
            lambda x: signal_statistic(x, sensor_name=get_sensor_type(sensor), Norm=True)
        ).unstack().reset_index()
        stats_df = stats_df.add_prefix(f"{sensor}_")
        stats_df.rename(columns={f"{sensor}_window_id": "window_id"}, inplace=True)
        features.append(stats_df)
    features_df = features[0]
    for other in features[1:]:
        features_df = pd.merge(features_df, other, on="window_id")
    return features_df

# === IMU ===
df_imu = pd.read_csv(imu_path)
sensors_imu = [col for col in df_imu.columns if col not in ["series_id", "label", "timestamp", "pkt_time_ns"]]
df_imu_windowed = sliding_window_df(df_imu, window_size_imu, overlap_ratio)
df_imu_features = dfFeatures(df_imu_windowed, sensors_imu)
df_imu_features[["label", "series_id"]] = df_imu_windowed.groupby("window_id")[["label", "series_id"]].first().reset_index(drop=True)
df_imu_features.to_csv(output_imu_path, index=False)

# === ADC ===
df_adc = pd.read_csv(adc_path)
sensors_adc = [col for col in df_adc.columns if col not in ["series_id", "label", "timestamp", "pkt_time_ns"]]
df_adc_windowed = sliding_window_df(df_adc, window_size_adc, overlap_ratio)
df_adc_features = dfFeatures(df_adc_windowed, sensors_adc)
df_adc_features[["label", "series_id"]] = df_adc_windowed.groupby("window_id")[["label", "series_id"]].first().reset_index(drop=True)
df_adc_features.to_csv(output_adc_path, index=False)

